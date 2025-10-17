#!/usr/bin/env python3

# Kabir Suri - Final Verison 

import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import String
import tf.transformations
from resit_coursework.srv import Room2Med, YOLOLastFrame
import time
import math
import actionlib



def move_to_goal(x, y, theta):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.get_rostime()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_state() == actionlib.GoalStatus.SUCCEEDED




def detect_person(): #function using 'yolo_ros' to detect different people

    rospy.wait_for_service('/detect_frame')

    try:
        detect_person_service = rospy.ServiceProxy('/detect_frame', YOLOLastFrame)
        response = detect_person_service("person")
        return response.detected

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False




def publish_tts(tts_pub, message, waiting_time):

    tts_pub.publish(String(data=message))
    time.sleep(waiting_time) # added a waiting time, for allowing the robot to start up, but also for inbetween decisions. Without it, at times skips speech.




def spin_360():

    rospy.loginfo("Preparing to spin and detect")

    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.angular.z = 1 # angular velocity
    rate = rospy.Rate(5) # Rate of 5 Hz

    # while spinning and detecting, it can take a long periods of time. Added a detection interval, to detect at certain times. Can reduce the value if need to detect every angle..
    detection_interval = 10
    person_detected = False

    steps = int(2 * math.pi / (1 / 5)) #Caluating full rotation using angular speed and ros rate.

    for i in range(steps):
        twist_pub.publish(twist)
        if i % detection_interval == 0:
            person_detected = person_detected or detect_person() # detects patient, and saves value if detected at any point.
            rospy.loginfo("Detecting...")
        rate.sleep()

    rospy.loginfo(f"Spinning has been completed, Person Found: {person_detected}")

    twist.angular.z = 0
    twist_pub.publish(twist)
    return person_detected



def get_medication(room_name): # calling the room2med service

    rospy.wait_for_service('room2med')

    try:
        room2med = rospy.ServiceProxy('room2med', Room2Med)
        response = room2med(room_name)
        rospy.loginfo(f"Medication response: {response.medication}")
        return response.medication

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None




# SMACHE CLASSES, ORIGINALLY HAD THEM IN SEPERATE FILES, BUT HAD IMPORTING PROBLEMS TOWARDS THE END, SO PUT ALL INTO MAIN
# TEMPORARY SOLUTION

class Initialise(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'])
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=10)

    def execute(self, userdata):

        rospy.sleep(2)
        publish_tts(self.tts_pub, "Booting up RoboKab", 1) # NAME OF ROBOT
        publish_tts(self.tts_pub, "Going to Doctor", 1)
        return 'succeeded'




class TalkToDoctor(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'])
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=10)

    def execute(self, userdata):

        publish_tts(self.tts_pub, "Hey Doctor, I hope you are well! I am going to start visiting each of the patients and asking them if they have taken their medication. I will see you soon.", 6)
        return 'succeeded'



class MoveToRoom(smach.State):

    def __init__(self, room_name, x, y, theta):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['room'])
        self.room_name = room_name
        self.x = x
        self.y = y
        self.theta = theta

    def execute(self, userdata):

        rospy.loginfo(f"Moving to {self.room_name}")
        userdata.room = self.room_name

        if move_to_goal(self.x, self.y, self.theta):
            return 'succeeded'

        else:
            return 'failed'



class SpinAndDetect(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['room', 'report'], output_keys=['report'])

    def execute(self, userdata):

        person_found = spin_360()

        if not person_found:
            userdata.report.append(f"Patient in {userdata.room}: not present, Medication taken: no")


        return 'succeeded' if person_found else 'failed'



class AskMedication(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['room', 'report', 'medication_taken'], output_keys=['medication_taken', 'report'])
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=10)

    def execute(self, userdata):

        #rospy.loginfo(f"Asking for medication in room: {userdata.room}")
        medication = get_medication(userdata.room)

        if medication:
            publish_tts(self.tts_pub, f"Hey Patient! Your medication is {medication}. Have you taken your medication?", 3)
            userdata.medication_taken = True

        else:
            userdata.medication_taken = False

        userdata.report.append(f"Patient in {userdata.room}: present, Medication taken: {'yes' if userdata.medication_taken else 'no'}")
        return 'succeeded'




class Report(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['report'])
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=10)

    def execute(self, userdata):

        doctor_detected = False
        count = 0

        while not doctor_detected and count <= 3:
            doctor_detected = spin_360()

            if doctor_detected:
                publish_tts(self.tts_pub, "Doctor Detected, Here are the reports:", 2)

                for entry in userdata.report:
                    publish_tts(self.tts_pub, entry, 2)
                return 'succeeded'

            else:
                publish_tts(self.tts_pub, "Doctor not detected. Waiting before trying again.", 2)
                rospy.sleep(5) # Wait for 5 seconds before retrying
                count = count + 1

        return 'failed'




def main():

    rospy.init_node('robokab')

    sm = smach.StateMachine(outcomes=['END'])
    sm.userdata.room = None
    sm.userdata.medication_taken = None
    sm.userdata.report = []

    with sm:

        smach.StateMachine.add('INITIALISE', Initialise(),
                               transitions={'succeeded':'MOVE_TO_DOCTOR'})

        smach.StateMachine.add('MOVE_TO_DOCTOR', MoveToRoom('doctor', 6.0, 8.5, 0.0),
                               transitions={'succeeded':'TALK_TO_DOCTOR', 'failed':'END'})

        smach.StateMachine.add('TALK_TO_DOCTOR', TalkToDoctor(),
                               transitions={'succeeded':'MOVE_TO_ROOM_C'})

        smach.StateMachine.add('MOVE_TO_ROOM_C', MoveToRoom('room_c', 10.6, 8.5, 0.0),
                               transitions={'succeeded':'SPIN_AND_DETECT_C', 'failed':'END'},
                               remapping={'room':'room'})

        smach.StateMachine.add('SPIN_AND_DETECT_C', SpinAndDetect(),
                               transitions={'succeeded':'ASK_MEDICATION_C', 'failed':'MOVE_TO_ROOM_F'},
                               remapping={'report':'report'})

        smach.StateMachine.add('ASK_MEDICATION_C', AskMedication(),
                               transitions={'succeeded':'MOVE_TO_ROOM_F'},
                               remapping={'room':'room', 'report':'report', 'medication_taken':'medication_taken'})

        smach.StateMachine.add('MOVE_TO_ROOM_F', MoveToRoom('room_f', 10.6, 2.5, 0.0),
                               transitions={'succeeded':'SPIN_AND_DETECT_F', 'failed':'END'},
                               remapping={'room':'room'})

        smach.StateMachine.add('SPIN_AND_DETECT_F', SpinAndDetect(),
                               transitions={'succeeded':'ASK_MEDICATION_F', 'failed':'MOVE_TO_REPORT'},
                               remapping={'report':'report'})

        smach.StateMachine.add('ASK_MEDICATION_F', AskMedication(),
                               transitions={'succeeded':'MOVE_TO_REPORT'},
                               remapping={'room':'room', 'report':'report', 'medication_taken':'medication_taken'})

        smach.StateMachine.add('MOVE_TO_REPORT', MoveToRoom('report', 6.0, 8.5, 1.57),
                               transitions={'succeeded':'REPORT', 'failed':'END'},
                               remapping={'room':'room'})

        smach.StateMachine.add('REPORT', Report(),
                               transitions={'succeeded': 'END', 'failed': 'END'})


    outcome = sm.execute()

if __name__ == '__main__':
    main()
