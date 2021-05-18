#!/usr/bin/env python

import rospy, sys
from std_msgs.msg import String
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged
from selenium import webdriver

USE_SPHINX = bool(int(sys.argv[1]))

'''
GPS for center of map   ( 35.233795, 129.082850 )
Parrot-Sphinx start GPS ( 8.878900, 2.367780 )
difference              ( -13.645105, 126.715070 )
'''

OFFSET_LAT = -13.645105
OFFSET_LON = 126.715070
drv = webdriver.Chrome(excutable_path = "/home/ubuntu1604/chromedriver")

def get_gps_cb(msg):
    if USE_SPHINX is True:
        lat  = msg.latitude + OFFSET_LAT
        long = msg.longitude + OFFSET_LON
        
    else:
        lat = msg.latitude
        long = msg.longitude
    
    print("latitude = %s, longitude = %s" %(lat, long))
    drv.execute_script("update_gps(%s, %s)" %(str(lat), str(long)))
   
if __name__ == '__main__':
    drv.get('http://localhost:8080')
    rospy.sleep(3)
    rospy.init_node("get_gps_position", anonymous = True)
    rospy.Subscriber("bebop/states/ardrone3/PilotingState/PositionChanged",
                     Ardrone3PilotingStatePositionChanged, get_gps_cb)
    rospy.spin()
                       
                      
