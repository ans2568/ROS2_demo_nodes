from navigation_interfaces.srv import Odom
from navigation_interfaces.srv import CurrentPose
from geometry_msgs.msg import PoseStamped

import rclpy

from flask import Flask, render_template, request
import os

template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates')
app = Flask(__name__, template_folder=template_dir)

def send_ros2_request(goto_client, current_x, current_y, current_z_angle, des_x, des_y, des_z_angle):

    # @TODO calculate z angle
    req = Odom.Request()
    req.departure = PoseStamped()

    req.departure.pose.position.x = current_x
    req.departure.pose.position.y = current_y
    req.departure.pose.position.z = 0.0
    req.departure.pose.orientation.x = 0.0
    req.departure.pose.orientation.y = 0.0
    req.departure.pose.orientation.z = current_z_angle
    req.departure.pose.orientation.w = 1.0

    req.destination = PoseStamped()
    req.destination.pose.position.x = des_x
    req.destination.pose.position.y = des_y
    req.destination.pose.position.z = 0.0
    req.destination.pose.orientation.x = 0.0
    req.destination.pose.orientation.y = 0.0
    req.destination.pose.orientation.z = des_z_angle
    req.destination.pose.orientation.w = 1.0

    req.init = True
    future = goto_client.call_async(req)

    return future

@app.route('/')
def hello():
    return render_template('home.html')

@app.route('/save', methods=['POST'])
def save():

    current_x = 0.0 if request.form['currentLocationX'] == '' else float(request.form['currentLocationX'])
    current_y = 0.0 if request.form['currentLocationY'] == '' else float(request.form['currentLocationY'])
    current_z_angle = 0.0 if request.form['currentLocationZ'] == '' else float(request.form['currentLocationZ'])
    
    des_x = 0.0 if request.form['destinationX'] == '' else float(request.form['destinationX'])
    des_y = 0.0 if request.form['destinationY'] == '' else float(request.form['destinationY'])
    des_z_angle = 0.0 if request.form['destinationZ'] == '' else float(request.form['destinationZ'])
    
    print(f'현재 위치 X: {current_x}, Y: {current_y}, Z: {current_z_angle} ,목적지 X: {des_x}, Y: {des_y}, Z: {des_z_angle}')

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('RESTInterface_Node')
    
    goto_client = node.create_client(Odom, 'initialization')
    while not goto_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('initialization service not available, waiting again ...')

    response = send_ros2_request(goto_client, current_x, current_y, current_z_angle, des_x, des_y, des_z_angle)
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