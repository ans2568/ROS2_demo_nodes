from navigation_interfaces.srv import Odom
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

class Client(Node):
	def __init__(self):
		super().__init__('client')
		self.go = self.create_client(Odom, 'navigation_service')
		while not self.go.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again ...')
		self.req = Odom.Request()

	def send_request(self):
		self.req.departure = PoseStamped()
		self.req.departure.pose.position.x = 0.343267
		self.req.departure.pose.position.y = -2.98099
		self.req.departure.pose.orientation.x = 0.0
		self.req.departure.pose.orientation.y = 0.0
		self.req.departure.pose.orientation.z = 0.99939
		self.req.departure.pose.orientation.w = 0.0349126

		self.req.destination = PoseStamped()
		self.req.destination.pose.position.x = 2.0
		self.req.destination.pose.position.y = 0.0
		self.req.destination.pose.orientation.z = 0.99939
		self.req.destination.pose.orientation.w = 0.0349126
		self.init = False
		self.future = self.go.call_async(self.req)

def main(args=None):
	rclpy.init(args=args)
	client = Client()
	client.send_request()
	rclpy.spin_once(client)
	if client.future.done():
		try:
			response = client.future.result()
		except Exception as e:
			print(e)
		else:
			print(response.result)

	client.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
