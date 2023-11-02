from navigation_interfaces.srv import Odom
from navigation_interfaces.srv import CurrentPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import String

import rclpy
from rclpy.node import Node

class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info('start control_node')

        self.initial_pose = PoseStamped()

        # Get Departure and Destination information from RESTInterfaceNode
        self.initialization_service = self.create_service(Odom, 'initialization', self.initialization)

        # Send Departure and Destination information to NavigationNode
        self.navigation_client = self.create_client(Odom, 'navigation_service')

        # Send Comeback Information to NavigationNode
        self.comeback_service = self.create_service(Odom, 'comeback_service', self.comeback_callback)

        # Get Current Pose Information from NavigationNode
        self.current_pose_client = self.create_client(CurrentPose, 'current_pose')

        # Send Stop request to NavigationNode
        self.stop_service = self.create_service(Empty, 'stop_request_service', self.stop_callback)

        # Send Food Mention request to Food Mention Node
        self.arrived_publisher = self.create_publisher(String, '/destination_arrived', 10)

    def initialization(self, request, response):
        # set initial pose and navigate to destination
        if request.init:
            self.initial_pose = request.departure
            self.final_pose = request.destination

        departure = request.departure
        destination = request.destination
        future = self.navigation(departure=departure, destination=destination, init=request.init)
        if future:
            try:
                nav_res = future.result()
            except Exception as e:
                print(e)
                response.result = 'navigation_error'
                self.get_logger().warn('Result of navigation_service: %s' % (response.result))
                return response
            else:
                if nav_res.result == 'success':
                    self.publish_arrived_destination()
                    response.result += 'publish /destination_arrived\n' + nav_res.result
                else:
                    response.result = nav_res.result
                self.get_logger().info('Result of navigation_service and food_mention_service : %s' % (response.result))
                return response
        else:
            response.result = 'no navigation service'
            self.get_logger().warn('Result of navigation_service: %s' % (response.result))
            return response

    def comeback_callback(self, _, response):
        future = self.get_current_pose()
        if future.done():
            try:
                res = future.result()
            except Exception as e:
                print(e)
                response.result = 'get_current_pose_error'
                self.get_logger().warn('Result of current_pose_service: %s' % (response.result))
                return response
            else:
                # Go to HOME
                if not self.initial_pose is None or not res.current_pose is None:
                    departure = res.current_pose
                    destination = self.initial_pose
                else:
                    response.result = 'No HOME Pose or No current_pose'
                    self.get_logger().warn('Result of current_pose_service: %s' % (response.result))
                    return response
                future = self.navigation(departure=departure, destination=destination)
                if future.done():
                    try:
                        res = future.result()
                    except Exception as e:
                        print(e)
                        response.result = 'navigation_error'
                        self.get_logger().warn('Result of navigation_service: %s' % (response.result))
                        return response
                    else:
                        response.result = res.result
                        self.get_logger().info('Result of navigation_service: %s' % (response.result))
                        return response

    def stop_callback(self, request, response):
        stop_client = self.create_client(Empty, 'stop service')
        request = Empty.Request()
        future = stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response

    def get_current_pose(self):
        req = CurrentPose.Request()
        future = self.current_pose_client.call_async(req)
        return future

    def navigation(self, departure, destination, init=False):
        req = Odom.Request()
        req.departure = departure
        req.destination = destination
        req.init = init
        goto_future = self.navigation_client.call_async(req)
        if goto_future.done():
            return goto_future
        else:
            return None

    def publish_arrived_destination(self):
        msg = String()
        msg.data = "The robot has reached the destination"
        self.arrived_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_service = ControlNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()