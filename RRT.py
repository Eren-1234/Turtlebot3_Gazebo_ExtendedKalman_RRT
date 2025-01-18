import rospy
import matplotlib.pyplot as plt
import random
import math
import copy
import numpy as np
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

show_animation = True

class RRT:
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, engeller, rand_area, expand_dis=1.0, goal_sample_rate=5, max_iter=500, buffer=0.5):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        engeller:engeller Positions [[x,y,size],...]
        rand_area:Random Sampling Area [min,max]
        buffer: Additional buffer distance around obstacles
        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.engeller = engeller
        self.buffer = buffer

    def planning(self):
        """
        Path planning
        """
        self.node_list = [self.start]
        while True:
            if random.randint(0, 100) > self.goal_sample_rate:
                rnd = [random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand)]
            else:
                rnd = [self.end.x, self.end.y]

            # En yakın düğümü bulmak için
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            nearest_node = self.node_list[nearest_ind]

            # Devamlı ağaç oluşturmak için
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
            new_node = copy.deepcopy(nearest_node)
            new_node.x += self.expand_dis * math.cos(theta)
            new_node.y += self.expand_dis * math.sin(theta)
            new_node.parent = nearest_ind

            if not self.collision_check(new_node):
                continue

            self.node_list.append(new_node)

            dx = new_node.x - self.end.x
            dy = new_node.y - self.end.y
            distance_to_goal = math.sqrt(dx ** 2 + dy ** 2)
            if distance_to_goal <= self.expand_dis:
                print("Başla!")
                break

            if show_animation:
                self.draw_graph(rnd)

        path = self.generate_final_path(len(self.node_list) - 1)
        return path

    def draw_graph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.node_list:
            if node.parent is not None:
                plt.plot([node.x, self.node_list[node.parent].x], [node.y, self.node_list[node.parent].y], "-g")

        for (ox, oy, size) in self.engeller:
            circle = plt.Circle((ox, oy), size + self.buffer, color="black", fill=True)
            plt.gca().add_artist(circle)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    def get_nearest_node_index(self, node_list, rnd):
        distances = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
        min_index = distances.index(min(distances))
        return min_index

    def collision_check(self, node):
        for (ox, oy, size) in self.engeller:
            dx = ox - node.x
            dy = oy - node.y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            if distance <= size + self.buffer:
                return False
        return True

    def generate_final_path(self, goal_index):
        path = [[self.end.x, self.end.y]]
        while self.node_list[goal_index].parent is not None:
            node = self.node_list[goal_index]
            path.append([node.x, node.y])
            goal_index = node.parent
        path.append([self.start.x, self.start.y])
        return path


class Node:
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main():
    rospy.init_node('rrt_planner')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Engelleri yükle
    engeller = []
    with open("engeller.txt", "r") as f:
        for _ in range(8):
            x = int(f.readline().strip())
            y = int(f.readline().strip())
            size = int(f.readline().strip())
            engeller.append((x, y, size))

    # RRT algoritmasını başlat
    rrt = RRT(start=[0, 0], goal=[4, 3], rand_area=[-15, 15], engeller=engeller, buffer=0.5)
    path = rrt.planning()

    if show_animation:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.show()

    # Robotun hareketi için başlangıç parametreleri
    robot_x, robot_y = 0.0, 0.0
    robot_theta = 0.0
    rate = rospy.Rate(10)
    twist = Twist()

    for goal_x, goal_y in path[::-1]:  # Yolu ters sırayla takip et
        while True:
            # Hedefe olan mesafe ve açı farkı
            dx = goal_x - robot_x
            dy = goal_y - robot_y
            distance_to_goal = math.sqrt(dx ** 2 + dy ** 2)
            goal_theta = math.atan2(dy, dx)
            angle_diff = goal_theta - robot_theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize edilmiş açı

            # Hedefe ulaşıldıysa bir sonraki noktaya geç
            if distance_to_goal < 0.1:
                print(f"Hedefe ulaşıldı: ({goal_x}, {goal_y})")
                break

            # Hız kontrolü
            twist.linear.x = min(0.2, distance_to_goal)  # Doğrusal hız
            twist.angular.z = 1.5 * angle_diff          # Açısal hız

            # Eğer açı farkı çok büyükse (90 derece veya daha fazla), sadece dön
            if abs(angle_diff) > math.pi / 4:
                twist.linear.x = 0.0

            # Komutları yayınla
            pub.publish(twist)

            # Robotun pozisyonunu güncelle
            dt = 0.1  # Zaman adımı
            robot_x += twist.linear.x * math.cos(robot_theta) * dt
            robot_y += twist.linear.x * math.sin(robot_theta) * dt
            robot_theta += twist.angular.z * dt

            # Robotun yol üzerindeki hareketini çizin
            if show_animation:
                rrt.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.plot(robot_x, robot_y, "ob")
                plt.pause(0.01)

    # Robotu durdur
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    print("Tüm hedeflere ulaşıldı, robot durdu.")


if __name__ == '__main__':
    main()
