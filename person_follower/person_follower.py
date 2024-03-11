# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges
        #
        # your code for computing vx, wz
        #

        def follow_target(np_ranges, target_angle=180):
            min_index = np.argmin(np_ranges)
            min_value = np.min(np_ranges)

            # Calcular el error (Pos está a la izquierda/ Neg está a la derecha)
            error_angle = target_angle - min_index
            
            # Control proporcional para ajustar las velocidades lineal y angular
            if abs(error_angle) <= 35 and min_value <=1.5:
                if abs(error_angle) <= 20:
                    angular_velocity = 0.2*(error_angle/90)
                    linear_velocity = (1 - np.exp(-0.693 * min_value))/2.4
                else:
                    angular_velocity = 2.5*(error_angle/90)
                    linear_velocity = (1 - np.exp(-0.693 * min_value))/3.2
            else:
                linear_velocity  = 0.
                angular_velocity = 0.                 

            # print("Ángulo:", min_index, "Distancia:", round(min_value, 3), "Vel Lin:", round(linear_velocity, 4), "Vel Ang:", round(angular_velocity, 4))
            print("error_angle:", round(error_angle, 3),"min_index:", round(min_index, 3),"min_value:", round(min_value, 3),"target_angle:", round(target_angle, 3))

            return linear_velocity, angular_velocity
        target_value = 180
        margin_value = 35
        np_ranges = np.array(ranges)
        np_ranges = np_ranges[target_value - margin_value:target_value + margin_value+1]
        linear_velocity, angular_velocity = follow_target(np_ranges,len(np_ranges)/2)

        vx = linear_velocity #0.1
        wz = angular_velocity #0.1 #pos izq, neg der
        #
        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
