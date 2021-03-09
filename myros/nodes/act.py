#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from robotclass.Motor import Motor
from robotclass.servo import Servo

# Costanti per le mosse dei messaggi
FORWARD = "forward"
LEFT = "left"
RIGHT = "right"
SERVO = "servo"
SEPARATOR = "-"
BACKWARD = "backward"
STOP = "stop"
END = "end"

class Act:
    
    motors = None
    servo = None
    servorotation = [33, 78, 123]
    
    def __init__(self):
        self.motors = Motor()
        self.servo = Servo()
        self.servo.setServoPwm('0', self.servorotation[1])
        self.servo.setServoPwm('1', 100)
    
    def execute(self, data):
        # in base al messaggio che ricevo, setto i motori o il servo0
        
        if data.data == FORWARD:            
            self.motors.setMotorModel(-800,-800,-800,-800)
             
        elif data.data == LEFT:
            self.motors.setMotorModel(1500,1500,-2500,-2500)
             
        elif data.data == RIGHT:
            self.motors.setMotorModel(-2500,-2500,1500,1500)
             
        elif data.data == (FORWARD + SEPARATOR + RIGHT):
            self.motors.setMotorModel(-4000,-4000,2000,2000)
             
        elif data.data == (FORWARD + SEPARATOR + LEFT):
            self.motors.setMotorModel(2000,2000,-4000,-4000)
             
        elif data.data == BACKWARD:
            self.motors.setMotorModel(800,800,800,800)
             
        elif data.data == STOP:
            self.motors.setMotorModel(0,0,0,0)
            
        elif data.data == (SERVO + SEPARATOR + RIGHT):
            self.servo.setServoPwm('0',self.servorotation[2])
            
        elif data.data == (SERVO + SEPARATOR + LEFT):
            self.servo.setServoPwm('0',self.servorotation[0])
            
        elif data.data == SERVO:
            self.servo.setServoPwm('0',self.servorotation[1])
            
        elif data.data == END:
            self.motors.destroy()
            self.servo.setServoPwm('0',self.servorotation[1])

            
         # rospy.loginfo(' Dal brain ho ricevuto l\'istruzione ' + data.data)

    def listenToBrain(self):
         # inizializzo il nodo act come listener sul topic 'azioni'
        rospy.Subscriber('azioni', String, self.execute)
         # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    
    try:
        rospy.init_node('action', anonymous=True)
        action = Act()
        action.listenToBrain()
    except KeyboardInterrupt:
        motors = Motor()
        motors.destroy()
