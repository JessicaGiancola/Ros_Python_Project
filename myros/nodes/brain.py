#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from myros.msg import Sensor

# Costanti per le mosse dei messaggi
FORWARD = "forward"
LEFT = "left"
RIGHT = "right"
SERVO = "servo"
SEPARATOR = "-"
BACKWARD = "backward"
STOP = "stop"
END = "end"

# threshold distanza dell'ostacolo dal sensore ultrasuoni
DISTANCE_FORWARD = 15
DISTANCE_LATERAL = 20

# costanti per stati
UNABLE_TO_MOVE = -1
INIT_STATE = 0
LINE_TRACKING = 1
CHECK_OBSTACLE = 2
AVOID_OBSTACLE = 3
END_SIMULATION = 4


class Brain:
    
    lastmove = None
    state = None
    servorotation = None
    turn = None
    
    
    def __init__(self):
        
        self.lastmove = ""
        self.state = INIT_STATE
        self.servorotation = 90
        
    def elaborates(self, data):
        # inizializzo il nodo brain come publisher sul topic 'azioni'
        pub = rospy.Publisher('azioni', String, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        message = String()
        
        if self.state == INIT_STATE:
            #valutare qual'è lo stato in base ai sensori
            
            if data.linetracking > 0 and data.ultrasonic > DISTANCE_FORWARD:
                self.state = LINE_TRACKING
            elif data.linetracking > 0 and data.ultrasonic <= DISTANCE_FORWARD:
                self.state = CHECK_OBSTACLE
            else:
                # impossibile definire lo stato iniziale, per cui non possiamo definire una mossa
                self.state = UNABLE_TO_MOVE
            
        else:
                    
            if self.state == LINE_TRACKING:
                
                if data.ultrasonic > DISTANCE_FORWARD:
                    #non ho ostacoli davanti quindi continuo a seguire la linea
                    
                    if data.linetracking==2:
                        message = FORWARD                
                    elif data.linetracking==4:
                        message = LEFT
                    elif data.linetracking==6:
                        message = FORWARD + SEPARATOR + LEFT
                    elif data.linetracking==1:
                        message = RIGHT
                    elif data.linetracking==3:
                        message = FORWARD + SEPARATOR + RIGHT  
                    elif data.linetracking==7:
                        message = STOP

                else:
                    # rilevo un ostacolo vicino, mi fermo
                    self.state = CHECK_OBSTACLE
                    message = STOP
                        
            elif self.state == CHECK_OBSTACLE:
                #  ho rilevato un ostacolo e ruoto i servo per decidere in quale direzione aggirarlo
                
                if self.servorotation == 90:
                
                    message = SERVO + SEPARATOR + LEFT
                    self.servorotation = 45
                    
                elif self.servorotation == 45:
                    
                    if data.ultrasonic > DISTANCE_LATERAL:
                    
                        self.state = AVOID_OBSTACLE
                        self.turn = LEFT
                        self.servorotation = 90
                        message = SERVO
                        
                    else:
                        
                        message = SERVO + SEPARATOR + RIGHT
                        self.servorotation = 135
                    
                elif self.servorotation == 135:
                    
                    if data.ultrasonic > DISTANCE_LATERAL:
                    
                        self.state = AVOID_OBSTACLE
                        self.turn = RIGHT
                        self.servorotation = 90
                        message = SERVO
                    
                    else:
                        # non posso aggirarlo, quindi termino il programma
                        self.state = END_SIMULATION
                        message = END
                    
            
            elif self.state == AVOID_OBSTACLE:
                # ho trovato una strada per aggirare l'ostacolo
                
                if self.turn != None:
                    
                    if self.turn == LEFT:
                        # aggiro l'ostacolo a sinistra
                        if self.lastmove == LEFT:
                            
                            message = RIGHT
                            self.state = LINE_TRACKING
                            self.turn = None
                            
                        else:
                            message = LEFT
                    
                    elif self.turn == RIGHT:
                        # aggiro l'ostacolo a destra
                        if self.lastmove == RIGHT:
                            
                            message = LEFT
                            self.state = LINE_TRACKING
                            self.turn = None
                            
                        else:
                            message = RIGHT
                            
            elif self.state == UNABLE_TO_MOVE:
                # tentiamo di reinizializzare lo stato all'iterazione successiva
                self.state = INIT_STATE

            elif self.state == END_SIMULATION:
                self.endSimulation()
           
        # mando i messaggi all'act se non ho terminato il programma
        if self.lastmove != END:
        
            self.lastmove = message    
            rospy.loginfo(" Ho ricevuto dai sensori: ultrasuono " + str(data.ultrasonic) + " line " + str(data.linetracking) + " e ho deciso " + str(message))
            pub.publish(message)
            rate.sleep()
            
        else:
            self.endSimulation()
            
    def endSimulation(self):
        rospy.signal_shutdown("La simulazione è terminata")
            

    def listenToSensors(self):
        # inizializzo il nodo come listener sul topic 'sensori'
        rospy.Subscriber('sensori', Sensor, self.elaborates)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
     try:
        brain = Brain()
        rospy.init_node('brain', anonymous=True)
        brain.listenToSensors()
     except rospy.ROSInterruptException:
        pass
