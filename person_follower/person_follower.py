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
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from scipy.optimize import curve_fit
import math 

counterState = 0
angular_velocity = 0.
linear_velocity = 0.

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
        global counterState
        global angular_velocity
        global linear_velocity

        def plot_subarrays(subarrays, indices, figsize=(6,6)):
            # Crear una nueva figura
            plt.figure(figsize=figsize)
            
            # Dibujar la cuadrícula polar
            ax = plt.subplot(111, polar=True)
            
            # Plotear los puntos dados en coordenadas polares
            for subarray, angle in zip(subarrays, indices):
                print(subarray)
                print(angle)
                theta = np.deg2rad(angle/3)  # Convertir el ángulo a radianes
                r = subarray  # Obtener las magnitudes
                ax.plot(theta, r, 'ro')  # 'ro' significa círculo rojo
            
            ax.set_theta_direction(-1)  # Cambiar la dirección de los ángulos (opcional)
            ax.set_theta_zero_location('N')  # Establecer la posición cero de los ángulos
            
            # Mostrar la imagen
            plt.show()


        # Función de parabola
        def parabola(x, a, b, c):
            return a * x**2 + b * x + c

        # Función de segmentación
        def create_subarrays(data):
            """
            Se debe filtrar cada subarray:
            - Por cantidad de puntos si es muy menor a determinada cantidad
            - Si no es parabola eliminar subarray
            """
            subarrays = []
            indices = []
            current_subarray = []
            current_indices = [] 
            for index,item in enumerate(data):
                if not math.isinf(float(item)):
                    current_subarray.append(float(item))
                    current_indices.append(index)
                else:
                    if current_subarray and len(current_subarray)> 5:
                        subarrays.append(np.array(current_subarray))
                        indices.append(current_indices)
                        current_subarray = []
                        current_indices = []

            if current_subarray:
                subarrays.append(np.array(current_subarray))
                indices.append(current_indices)
            print(len(subarrays),len(indices))

            plot_subarrays(subarrays, indices)
            return subarrays, indices

        # Identifica pierna
        def isLeg(np_ranges):
            # min_index = np.argmin(np_ranges)
            leg = np.array(np_ranges)#[min_index-10:min_index+10]
            leg = leg[~np.isnan(leg) & ~np.isinf(leg)]

            # Eje x correspondiente a los índices de los puntos
            x_data = np.arange(len(leg))
            is_semicircle = False
            if (x_data != []):
            # Ajuste de los puntos a la función de la circunferencia
                popt, _ = curve_fit(parabola, x_data, leg)

                error = np.sqrt(np.mean((leg - parabola(x_data, *popt))**2))
                umbral_error = 0.005
                is_semicircle = error < umbral_error

                print(f"Error({is_semicircle}): ", f"{round(error,5)}/{round(umbral_error,5)}")

            return is_semicircle

        def follow_target(np_ranges, target_angle):
            """
            Se podría mejorar para que solo con el valor de entrada del target_angle lo siga
            Sin necesidad del arreglo np_ranges.
            """
            min_index = np.argmin(np_ranges)
            min_value = np.min(np_ranges)

            error_angle = target_angle - min_index
            
            # Control proporcional para ajustar las velocidades lineal y angular
            if abs(error_angle) <= 30*3 and min_value <= 2:
                if abs(error_angle) <= 15*3:
                    angular_velocity = 0.2*(error_angle/90)
                    linear_velocity = (1 - np.exp(-0.7 * min_value))/2.4
                else:
                    angular_velocity = 2.5*(error_angle/90)
                    linear_velocity = (1 - np.exp(-0.7 * min_value))/3.2
            else:
                linear_velocity  = 0.
                angular_velocity = 0.                 

            return linear_velocity, angular_velocity

        margin_value = 30*3
        target_value = 180*3

        np_ranges = np.array(ranges)
        # np_ranges = np.roll(np_ranges, margin_value)[:margin_value * 2]
        subarrays = create_subarrays(np_ranges)
        # isLegValue = isLeg(np_ranges)

        # if (isLegValue):
        #     counterState = 0
        #     linear_velocity, angular_velocity = follow_target(np_ranges,len(np_ranges)/2)
        # else:
        #     counterState += 1
        #     if (counterState > 10):
        #         print('Encuentra un muro')
        #         linear_velocity  = 0.
        #         angular_velocity = 0. 

        vx = linear_velocity #0.1
        wz = angular_velocity #0.1 #pos izq, neg der
        #
        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = -wz
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
