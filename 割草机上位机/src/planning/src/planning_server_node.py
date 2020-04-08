#!/usr/bin/env python
# license removed for brevity
import rospy

try:
    import route_plan_server
except:
    from planning_application import route_plan_server

try:
    import coverage_server
except:
    from planning_application import coverage_server

try:
    import along_edge_server
except:
    from planning_application import along_edge_server

def Planning_Server():
    route_plan_server.Route_Plan_Server()
    coverage_server.Subimage_Converage_Server()
    along_edge_server.Along_Edge_Server()
    return

if __name__ == '__main__':
    try:
        rospy.init_node('planning_node', anonymous=True)
        Planning_Server()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass