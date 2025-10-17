#!/usr/bin/env python3

import rospy
from resit_coursework.srv import Room2Med, Room2MedResponse

room_meds = {
    "room_c": "Paracetamol",
    "room_f": "Ibuprofin",
}

def call_back(req):
    medication = room_meds.get(req.room_name, "N/A")
    #rospy.loginfo(f"Returning meds: {medication}")
    return Room2MedResponse(medication)

def room2med_server():
    rospy.init_node('room2med_server')
    s = rospy.Service('room2med', Room2Med, call_back)
    rospy.loginfo("Service 'room2med' is ready")
    rospy.spin()

if __name__ == "__main__":
    room2med_server()