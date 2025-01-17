import rospy
import numpy as np
import math
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu

class ExtendedKalmanFilterNode:
    def __init__(self):
        rospy.init_node("ekf_node", anonymous=True)

        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)

        self.pub = rospy.Publisher('kalman_filtered_pose', Int32MultiArray, queue_size=10)

        # GPS Origin for Local Coordinates
        self.origin_latitude = 40.9955106085895
        self.origin_longitude = 29.062988181734983

        # İlk değerler
        self.robot_latitude = 0
        self.robot_longitude = 0
        self.x_acceleration = 0
        self.y_acceleration = 0
        self.dt = 0.5

        # Durum Vector ve Matrisleri
        self.state = np.matrix([[0], [0], [0], [0]])  # [x, y, x_dot, y_dot]
        self.F = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        self.G = np.matrix([[0.5 * self.dt ** 2, 0],
                            [0, 0.5 * self.dt ** 2],
                            [self.dt, 0],
                            [0, self.dt]])
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
        self.Q = np.eye(4) * 0.001
        self.R = np.eye(2) * 0.002
        self.P = np.eye(4) * 0.002

    def gps_callback(self, msg):
        self.robot_latitude = msg.latitude
        self.robot_longitude = msg.longitude

    def imu_callback(self, msg):
        self.x_acceleration = msg.linear_acceleration.x
        self.y_acceleration = msg.linear_acceleration.y

    def convert_to_local_coordinates(self, lat, lon):
        WORLD_POLAR_M = 6356752.3142
        WORLD_EQUATORIAL_M = 6378137.0

        eccentricity = math.acos(WORLD_POLAR_M / WORLD_EQUATORIAL_M)
        n_prime = 1 / (math.sqrt(1 - (math.sin(math.radians(lat)) ** 2) * (math.sin(eccentricity) ** 2)))
        m = WORLD_EQUATORIAL_M * (math.cos(eccentricity) ** 2) * (n_prime ** 3)
        n = WORLD_EQUATORIAL_M * n_prime

        diff_lon = lon - self.origin_longitude
        diff_lat = lat - self.origin_latitude

        surf_dist_lon = (math.pi / 180) * math.cos(math.radians(lat)) * n
        surf_dist_lat = (math.pi / 180) * m

        x = diff_lon * surf_dist_lon
        y = diff_lat * surf_dist_lat

        return x, y

    def predict(self):
        accel = np.matrix([[self.x_acceleration], [self.y_acceleration]])
        predicted_state = self.F * self.state + self.G * accel
        predicted_covariance = self.F * self.P * self.F.T + self.Q

        return predicted_state, predicted_covariance

    def correct(self, predicted_state, predicted_covariance, measurement):
        S = self.H * predicted_covariance * self.H.T + self.R
        K = predicted_covariance * self.H.T * np.linalg.inv(S)

        self.state = predicted_state + K * (measurement - self.H * predicted_state)
        self.P = (np.eye(self.H.shape[1]) - K * self.H) * predicted_covariance

        return float(self.state[0]), float(self.state[1])

    def run(self):
        rate = rospy.Rate(2)  

        while not rospy.is_shutdown():
            x_local, y_local = self.convert_to_local_coordinates(self.robot_latitude, self.robot_longitude)
            measurement = np.matrix([[x_local], [y_local]])

            predicted_state, predicted_covariance = self.predict()
            x, y = self.correct(predicted_state, predicted_covariance, measurement)

            pose_message = Int32MultiArray()
            pose_message.data = [x, y]
            self.pub.publish(pose_message)

            rospy.loginfo(f"Filtered Position -> x: {x:.2f}, y: {y:.2f}")
            rate.sleep()

if __name__ == "__main__":
    try:
        ekf_node = ExtendedKalmanFilterNode()
        ekf_node.run()
    except rospy.ROSInterruptException:
        pass
