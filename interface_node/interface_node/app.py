from navigation_interfaces.srv import Odom
from navigation_interfaces.srv import CurrentPose
from geometry_msgs.msg import PoseStamped

import rclpy

from flask import Flask, render_template, request
import os
import yaml

template_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'templates/')
app = Flask(__name__, template_folder=template_dir)

def send_ros2_request(goto_client, current_x, current_y, current_quaternion_z, current_quaternion_w, des_x, des_y, des_quaternion_z, des_quaternion_w):

    req = Odom.Request()
    req.departure = PoseStamped()

    req.departure.pose.position.x = current_x
    req.departure.pose.position.y = current_y
    req.departure.pose.position.z = 0.0
    req.departure.pose.orientation.x = 0.0
    req.departure.pose.orientation.y = 0.0
    req.departure.pose.orientation.z = current_quaternion_z
    req.departure.pose.orientation.w = current_quaternion_w

    req.destination = PoseStamped()
    req.destination.pose.position.x = des_x
    req.destination.pose.position.y = des_y
    req.destination.pose.position.z = 0.0
    req.destination.pose.orientation.x = 0.0
    req.destination.pose.orientation.y = 0.0
    req.destination.pose.orientation.z = des_quaternion_z
    req.destination.pose.orientation.w = des_quaternion_w

    req.init = True
    future = goto_client.call_async(req)

    return future

@app.route('/')
def hello():
    return render_template('home.html')

@app.route('/get_data')
def get_yaml():
    file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'static/map/map')
    yaml_file = file + '.yaml'
    pgm_file = file + '.pgm'
    data = []
    with open(yaml_file, 'r') as y:
        yaml_data = yaml.load(y, Loader=yaml.FullLoader)
        data.extend([float(yaml_data['origin'][0]), float(yaml_data['origin'][1]), float(yaml_data['resolution'])])
    with open(pgm_file, 'rb') as p:
        # 이진 PGM 파일인 경우
        p.readline()  # P5
        dimensions = p.readline().decode('utf-8').split()
        _, height = map(int, dimensions)
        data.extend([height])
    return list(data)

@app.route('/navigation', methods=['POST'])
def navigation():
    data = request.get_json()
    current_x = 0.0 if data.get('initPositionX', 0.0) == '' else float(data.get('initPositionX', 0.0))
    current_y = 0.0 if data.get('initPositionY', 0.0) == '' else float(data.get('initPositionY', 0.0))
    current_quaternion_z = 0.0 if data.get('initQuaternionZ', 0.0) == '' else float(data.get('initQuaternionZ', 0.0))
    current_quaternion_w = 0.0 if data.get('initQuaternionW', 0.0) == '' else float(data.get('initQuaternionW', 0.0))
    
    des_x = 0.0 if data.get('destPositionX', 0.0) == '' else float(data.get('destPositionX', 0.0))
    des_y = 0.0 if data.get('destPositionY', 0.0) == '' else float(data.get('destPositionY', 0.0))
    des_quaternion_z = 0.0 if data.get('destQuaternionZ', 0.0) == '' else float(data.get('destQuaternionZ', 0.0))
    des_quaternion_w = 0.0 if data.get('destQuaternionW', 0.0) == '' else float(data.get('destQuaternionW', 0.0))

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('RESTInterface_Node')

    goto_client = node.create_client(Odom, 'initialization')
    idx = 0
    while not goto_client.wait_for_service(timeout_sec=1.0):
        idx += 1
        node.get_logger().info('initialization service not available, waiting again ...')
        if idx > 10:
            return render_template('home.html')

    response = send_ros2_request(goto_client, current_x, current_y, current_quaternion_z, current_quaternion_w, des_x, des_y, des_quaternion_z, des_quaternion_w)
    rclpy.spin_once(node)
    if response.done():
        try:
            response = response.result()
        except Exception as e:
            node.get_logger().info(
                'Service call failed %r' % (e))
        else:
            node.get_logger().info(
                'Result of service: %s' % (response.result))
    else:
        print('none response')
    rclpy.shutdown()
    return render_template('home.html')

def main():
    rclpy.init(args=None)
    app.run('0.0.0.0', port=3306, debug=True)