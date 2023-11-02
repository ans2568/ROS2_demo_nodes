from .BasicNavigator import BasicNavigator
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from navigation_interfaces.srv import Odom, CurrentPose
from std_srvs.srv import Empty

class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__('Navigation')
        self.nav_service = self.create_service(Odom, 'navigation_service', self.navigation)
        self.comeback_service = self.create_service(Odom, 'comeback_service', self.comeback)
        self.current_pose_service = self.create_service(CurrentPose, 'current_pose', self.current_pose_callback)
        self.stop_service = self.create_service(Empty, 'stop_service', self.stop_callback)
        self.current_pose = None
        self.nav = BasicNavigator()

    def navigation(self, request, response):
        departure = request.departure
        destination = request.destination

        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.nav.get_clock().now().to_msg()
        init_pose.pose = departure.pose
        self.nav.setInitialPose(initial_pose=init_pose)

        self.nav.waitUntilNav2Active()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose = destination.pose

        self.nav.goToPose(goal_pose)

        i = 0
        while not self.nav.isNavComplete():
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback.current_pose:
                self.current_pose = feedback.current_pose
            if feedback and i % 5 == 0:
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.nav.cancelNav()

        result = self.nav.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            response.result = 'success'
            print('Goal succeeded!')
        elif result == GoalStatus.STATUS_CANCELED:
            response.result = 'canceled'
            print('Goal canceled')
        elif result == GoalStatus.STATUS_ABORTED:
            response.result = 'failed'
            print('Goal failed')
        else:
            response.result = 'unknown'
            print('Goal has an invalid return status')
        return response

    def comeback(self, request, response):
        self.navigation(request=request, response=response)
        return response
    
    def current_pose_callback(self,_, response):
        if self.current_pose:
            response.current_pose = self.current_pose
            return response

    def stop_callback(self, _, response):
        if self.nav:
            self.nav.cancelNav()
            response = Empty.Response()
            return response
        else:
            print('no self.nav')

def main(args=None):
    rclpy.init(args=args)
    navigation = NavigationNode()
    rclpy.spin(navigation)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
