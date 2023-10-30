from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

class StopClient(Node):
	def __init__(self):
		super().__init__('stop_client')
		self.go = self.create_client(Empty, 'stop_service')
		while not self.go.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again ...')
		self.req = Empty.Request()

	def send_request(self):
		self.future = self.go.call_async(self.req)

def main(args=None):
	rclpy.init(args=args)
	client = StopClient()
	client.send_request()
	rclpy.spin_once(client)
	if client.future.done():
		try:
			response = client.future.result()
		except Exception as e:
			print(e)
		else:
			print('finish command')

	client.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()