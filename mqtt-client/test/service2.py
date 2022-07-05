from std_srvs.srv import Trigger

import rclpy


def handle_srv(request, response):
    response.success=True
    response.message="success"
    return response

rclpy.init()
node = rclpy.create_node('test_server')
srv = node.create_service(Trigger, 'local_service', handle_srv)

rclpy.spin(node)
rclpy.shutdown()
