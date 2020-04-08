#!/usr/bin/env python
# license removed for brevity
import rospy
try:
    import build_map
except:
    from map_application import build_map
try:
    import map_provided
except:
    from map_application import map_provided

def Map_Action_Server():
    build_map.Build_Map_Server_Node()
    map_provided.Map_Provided_Server_Node()
    return

if __name__ == '__main__':
    try:
        rospy.init_node('map_server_node', anonymous=True)
        Map_Action_Server()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass