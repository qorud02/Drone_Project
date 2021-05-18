#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from bb2_pkg.GetChar import GetChar
 
msg = """
---------------------------------------------------
 1:take off, 2:landing, 3:emergency, sp:stop(hover)
---------------------------------------------------
        w                           i                      
   a    s    d                j     k     l
---------------------------------------------------
w/s : up  / down           i/k : foward / backword
a/d : ccw / cw             j/l : left   / righ
---------------------------------------------------
-/+ : decrease / increase linear  speed by 10%
,/. : decrease / increase angular speed by 10%
---------------------------------------------------
CTRL-C to quit
"""

e = "Communications Failed"
 
moveBindings = {
    'w':( 0, 0, 0, 1), 'a':( 0, 0, 1, 0), 'i':( 1, 0, 0, 0), 'j':( 0, 1, 0, 0), 
    's':( 0, 0, 0,-1), 'd':( 0, 0,-1, 0), 'k':(-1, 0, 0, 0), 'l':( 0,-1, 0, 0),
    ' ':( 0, 0, 0, 0)
}

speedBindings = {
    '-':(0.9,1.0), '+':(1.1,1.0), ',':(1.0,0.9), '.':(1.0,1.1)
}


class MoveBebop():

    def __init__(self):
    
        rospy.init_node('bebop_teleop_key')
        
        self.pub0 = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        self.pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/bebop/land',    Empty, queue_size = 1)
        self.pub3 = rospy.Publisher('/bebop/reset',   Empty, queue_size = 1)
        
        self.empty_msg = Empty()
        self.key_input = GetChar()
        
        self.lin_spd = rospy.get_param("~speed", 0.5)
        self.ang_spd = rospy.get_param("~turn",  1.0)
        
        self.x     = 0
        self.y     = 0
        self.z     = 0
        self.th    = 0
        self.count = 0
        
    def get_speed(self, lin, ang):
        return "current speed:\tlinear = %s, angular = %s " % (lin, ang)


if __name__ == '__main__': 
    try:
        mb = MoveBebop()
        
        print(msg)
        print(mb.get_speed(mb.lin_spd, mb.ang_spd))
        
        key = ' '
        
        while key != 'Q':   # while not rospy.is_shutdown():
        
            key = mb.key_input.getch()
        
            if   key == '1':
                mb.pub1.publish(mb.empty_msg)
                
            elif key == '2':
                mb.pub2.publish(mb.empty_msg)
                
            elif key == '3':
                mb.pub3.publish(mb.empty_msg)
                
            elif key in moveBindings.keys():
                mb.x  = moveBindings[key][0]
                mb.y  = moveBindings[key][1]
                mb.z  = moveBindings[key][2]
                mb.th = moveBindings[key][3]
                
            elif key in speedBindings.keys():
                mb.lin_spd = mb.ang_spd * speedBindings[key][0]
                mb.ang_spd = mb.ang_spd * speedBindings[key][1]
 
                print(get_speed(lin_spd, ang_spd))
                
            else:
                pass
                    
            mb.count = (mb.count + 1) % 15
            
            if (mb.count == 14):
                print(msg)
 
            twist = Twist()
            
            twist.linear.x  = mb.x  * mb.lin_spd
            twist.linear.y  = mb.y  * mb.lin_spd
            twist.linear.z  = mb.th * mb.lin_spd
            
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = mb.z  * mb.ang_spd
            
            mb.pub0.publish(twist)
            
        mb.pub2.publish(mb.empty_msg)
 
    except rospy.ROSInterruptException:
        mb.pub2.publish(mb.empty_msg)
