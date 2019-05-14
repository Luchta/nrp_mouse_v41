#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nrp_mouse_v2.msg import nrpmouse_msg
  
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

#1 fla1	(Foreleft Shoulder servo)    	SB: 8
#2 fla2 (Foreleft Elbow servo)  	SB: 9
#3 fra1	(Foreright Shoulder servo)   	SB: 10
#4 fra2	(Foreright Elbow servo)  	SB: 11
#5 hla1	(Hindleft hip servo)    	SB: 0
#6 hla2    (Hindleft knee servo)   	SB: 1
#7 hra1	(Hindright hip servo)   	SB: 2
#8 hra2	(Hindright knee servo)  	SB: 3
#9 spine                           	SB: 4
#10 tail                            	SB: 5
#Spine2   (up/down)              	SB: 7
#Head    (left/right)            	SB: 12
#Head    (up/down)               	SB: 13

def messageCb(msgarr)
kit.servo[0].angle = msgarr[5] #hla1
kit.servo[1].angle = msgarr[6] #hla2
kit.servo[2].angle = msgarr[7] #hra1
kit.servo[3].angle = msgarr[8] #hra2
kit.servo[4].angle = msgarr[9] #spine
kit.servo[5].angle = msgarr[10] #tail
kit.servo[8].angle = msgarr[1] #fla1
kit.servo[9].angle = msgarr[2] #fla2
kit.servo[10].angle = msgarr[3] #fra1
kit.servo[11].angle = msgarr[4] #fra2


      
def listener():
  
      # In ROS, nodes are uniquely named. If two nodes with the same
      # name are launched, the previous one is kicked off. The
      # anonymous=True flag means that rospy will choose a unique
      # name for our 'listener' node so that multiple listeners can
      # run simultaneously.
     rospy.init_node('listener', anonymous=False)
  
      rospy.Subscriber("nrpmouse_servotopic", nrpmouse_msg, messageCb)
  
      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()
  
if __name__ == '__main__':
	listener()

