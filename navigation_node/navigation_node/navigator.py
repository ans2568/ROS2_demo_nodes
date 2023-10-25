from .BasicNavigator import BasicNavigator
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from navigation_interfaces.srv import Odom

class NavigationNode(Node):
	def __init__(self) -> None:
		super().__init__('Navigation')
		self.nav_service = self.create_service(Odom, 'goto_client', self.navigation)

	def navigation(self, request, respones):
		nav = BasicNavigator()
		departure = request.departure
		destination = request.destination

		init_pose = PoseStamped()
		init_pose.header.frame_id = 'map'
		init_pose.header.stamp = nav.get_clock().now().to_msg()
		init_pose.pose = departure.pose
		nav.setInitialPose(initial_pose=init_pose)

		nav.waitUntilNav2Active()

		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'map'
		goal_pose.header.stamp = nav.get_clock().now().to_msg()
		goal_pose.pose = destination.pose

        # sanity check valid path exist
        # path = nav.getPath(init_pose, goal_pose)
        # smooth_path = nav.smoothPath(path)
        # if (not smooth_path):
        #     print('There is no path to navigation')
        #     respones.result = 'There is no path to navigation'

        # else:
		nav.goToPose(goal_pose)

		i = 0
		while not nav.isNavComplete():
			i = i + 1
			feedback = nav.getFeedback()
			if feedback and i % 5 == 0:
				if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
					nav.cancelTask()

		result = nav.getResult()
		if result == GoalStatus.STATUS_SUCCEEDED:
			respones.result = 'success'
			print('Goal succeeded!')
		elif result == GoalStatus.STATUS_CANCELED:
			respones.result = 'canceled'
			print('Goal canceled')
		elif result == GoalStatus.STATUS_ABORTED:
			respones.result = 'failed'
			print('Goal failed')
		else:
			print(result)
			respones.result = 'unknown'
			print('Goal has an invalid return status')
		return respones

def main(args=None):
	rclpy.init(args=args)
	navigation = NavigationNode()
	rclpy.spin(navigation)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
