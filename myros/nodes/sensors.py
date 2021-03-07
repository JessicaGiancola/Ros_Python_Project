#!/usr/bin/env python

import rospy
from myros.msg import Sensor
import time
from robotclass.Ultrasonic import Ultrasonic
from robotclass.Line_Tracking import Line_Tracking


class Sensors:
    
    ultrasonic = None
    linetrack = None
    
    def __init__(self):
        
        self.ultrasonic = Ultrasonic()
        self.linetrack = Line_Tracking()
        
    def detect(self):
        # inizializzo il nodo dei sensori come publisher sul topic 'sensori'
        pub = rospy.Publisher('sensori', Sensor, queue_size=10)
        rate = rospy.Rate(3) # 3hz
        
        while not rospy.is_shutdown():
            # invio il messaggio sul topic
  
            message = Sensor()
            message.ultrasonic = self.ultrasonic.get_distance()
            message.linetracking = self.linetrack.run()
            rospy.loginfo(" messaggio " + str(message))
            pub.publish(message)
            rate.sleep()
            

if __name__ == '__main__':
    try:
        sense = Sensors()
        rospy.init_node('sensors', anonymous=True)
        sense.detect()
        
    except rospy.ROSInterruptException:
        pass
