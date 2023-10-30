from navigation_interfaces.srv import Odom
from navigation_interfaces.srv import CurrentPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

class ControlNode(Node):

    def __init__(self):
        super().__init__('minimal_service')
        # Get Departure and Destination information from RESTInterfaceNode
        self.initialization_service = self.create_service(Odom, 'initialization', self.initialization)
        self.initial_pose = PoseStamped()

        # Send Departure and Destination information to NavigationNode
        self.navigation_client = self.create_client(Odom, 'navigation_service')

        # Send Comeback Information to NavigationNode
        self.comeback_service = self.create_service(Odom, 'comeback_service', self.comeback_callback)

        # Get Current Pose Information from NavigationNode
        self.current_pose_client = self.create_client(CurrentPose, 'current_pose')

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

def main(args=None):
    rclpy.init(args=args)

    minimal_service = ControlNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()