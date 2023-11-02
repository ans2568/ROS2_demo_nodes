from navigation_interfaces.srv import Odom
from navigation_interfaces.srv import CurrentPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

class ControlNode(Node):

    def __init__(self):
        super().__init__('minimal_service')
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
        # @TODO mention_service change srv type Empty to Mention srv
        self.mention_client = self.create_client(Empty, 'mention_service')

    def initialization(self, request, response):
        # set initial pose and navigate to destination
        if request.init:
            self.initial_pose = request.departure
            self.final_pose = request.destination

        departure = request.departure
        destination = request.destination
        future = self.navigation(departure=departure, destination=destination, init=request.init)

        if future.done():
            try:
                nav_res = future.result()
            except Exception as e:
                print(e)
                response.result = 'navigation_error'
                return response
            else:
                if nav_res.result == 'success':
                    response.result = self.mention()
                    response.result += '\n' + nav_res.result
                else:
                    response.result = nav_res.result
        return response

    def comeback_callback(self, _, response):
        future = self.get_current_pose()
        if future.done():
            try:
                res = future.result()
            except Exception as e:
                print(e)
                response.result = 'get_current_pose_error'
                return response
            else:
                # Go to HOME
                if not self.initial_pose is None or not res.current_pose is None:
                    departure = res.current_pose
                    destination = self.initial_pose
                else:
                    response.result = 'No HOME Pose or No current_pose'
                    return response
                future = self.navigation(departure=departure, destination=destination)
                if future.done():
                    try:
                        res = future.result()
                    except Exception as e:
                        print(e)
                        response.result = 'navigation_error'
                        return response
                    else:
                        response.result = res.result
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
        future = self.current_pose_client.call_sync(req)
        return future

    def navigation(self, departure, destination, init=False):
        req = Odom.Request()
        req.departure = departure
        req.destination = destination
        req.init = init
        goto_future = self.navigation_client.call_sync(req)
        return goto_future

    def mention(self):
        response = None
        while not self.mention_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mention service not available, waiting again ...')

        # @TODO Instanciate Mention srv request
        mention_req = Empty()
        mention_future = self.mention_client.call_async(mention_req)
        rclpy.spin_until_future_complete(self, mention_future)
        if mention_future.done():
            try:
                mention_result = mention_future.result()
            except Exception as e:
                print(e)
            else:
                response = mention_result.result
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = ControlNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()