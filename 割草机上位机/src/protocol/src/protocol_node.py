#!/usr/bin/env python
# license removed for brevity

import rospy
try:
    import protocol_publish_node
except:
    from protocol_application import protocol_publish_node

try:
    import protocol_service_node
except:
    from protocol_application import protocol_service_node


if __name__ == '__main__':
    try:
        rospy.init_node('protocol_process_node', anonymous=True)
        protocol_service_node.Server_Init()
        protocol_publish_node.Publisher_Init()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass
