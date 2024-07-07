#!/usr/bin/env python3  

from tork_moveit_tutorial import *

import rospy, math


GroupName = "rarm"
TargetPosition1 = [ 0.3, -0.3, 1.6 ]
TargetPosition2 = [ 0.3, -0.3, 1.0 ]
TargetPosition3 = [ 0.3, 0.0, 1.0 ]
TargetPosition4 = [ 0.3, 0.0, 1.6 ]

def main():
    init_node()
    
    group = MoveGroupCommander(GroupName)

    while not rospy.is_shutdown():
        # End Effector Target 1
        rospy.loginfo( "Start End Effector Target 1")
        group.set_position_target( TargetPosition1 )
        group.go()

        # End Effector Target 2
        rospy.loginfo( "Start End Effector Target 2")
        group.set_position_target( TargetPosition2 )
        group.go()

        # End Effector Target 3
        rospy.loginfo( "Start End Effector Target 3")  
        group.set_position_target( TargetPosition3 )
        group.go()

        # End Effector Target 4
        rospy.loginfo( "Start End Effector Target 4")
        group.set_position_target( TargetPosition4 )
        group.go()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

