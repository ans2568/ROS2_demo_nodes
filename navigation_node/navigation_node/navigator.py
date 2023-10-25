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
		print(departure)
		print(destination)

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
		while not nav.isTaskComplete():
			i = i + 1
			feedback = nav.getFeedback()
			if feedback and i % 5 == 0:
				print('Estimated time of arrival: ' + '{0:.0f}'.format(
					Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
					+ ' seconds.')

				if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
					nav.cancelTask()

		result = nav.getResult()
		if result == GoalStatus.SUCCEEDED:
			print('Goal succeeded!')
			respones.result = 'success'
		elif result == GoalStatus.CANCELED:
			print('Goal canceled')
			respones.result = 'canceled'
		elif result == GoalStatus.FAILED:
			print('Goal failed')
			respones.result = 'failed'
		else:
			respones.result = 'unknown'
			print('Goal has an invalid return status')

def main(args=None):
	rclpy.init(args=args)
	navigation = NavigationNode()
	rclpy.spin(navigation)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
