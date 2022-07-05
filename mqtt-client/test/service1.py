from __future__ import print_function

from std_srvs.srv import Trigger, TriggerResponse
import rospy

def handle_srv(request):
    return TriggerResponse(True, "success")

rospy.init_node('test_server')
s = rospy.Service('local_service', Trigger, handle_srv)
rospy.spin()
