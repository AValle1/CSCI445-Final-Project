from pyCreate2 import create2
import math
import random
import odometry
import pd_controller
import pid_controller
import lab8_map
import rrt_map
import particle_filter
import rrt
import numpy as np
from enum import Enum


class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.arm = factory.create_kuka_lbr4p()
        self.virtual_create = factory.create_virtual_create()
        # self.virtual_create = factory.create_virtual_create("192.168.1.XXX")
        self.odometry = odometry.Odometry()
        self.mapJ = lab8_map.Map("lab8_map.json")
        self.map = rrt_map.Map("configuration_space.png")
        self.rrt = rrt.RRT(self.map)
        self.path = []

        # TODO identify good PID controller gains
        self.pd_controller = pd_controller.PDController(1000, 100, -75, 75)
        self.pdWF = pid_controller.PIDController(200, 50, 0, [0,0], [-50, 50])
        self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        self.pidTheta2 = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter(self.mapJ, 1000, 0.06, 0.15, 0.2)

        self.joint_angles = np.zeros(7)

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
            math.cos(goal_theta - self.odometry.theta))) > 0.05:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def go_to_goal(self, goal_x, goal_y):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2)) > 0.1:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            output_distance = self.pidDistance.update(0, distance, self.time.time())
            self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def take_measurements(self):
        angle = -90
        while angle <= 90:
            self.servo.go_to(angle)
            self.time.sleep(2.0)
            distance = self.sonar.get_distance()
            print(distance)
            self.pf.measure(distance, math.radians(angle))

            self.visualize()

            angle += 45
        self.servo.go_to(0)

    def localize_self(self):
        pos = self.create.sim_get_position()
        x_actual = pos[0]
        y_actual = pos[1]
        x, y, theta = self.pf.get_estimate()

        while math.sqrt(math.pow(x_actual - x, 2) + math.pow(y_actual - y, 2)) > 0.1:
            self.go_to_angle(self.odometry.theta + math.radians(20))
            distance = self.sonar.get_distance()
            self.pf.measure(distance, 0)
            self.visualize()
            #self.take_measurements()
            pos = self.create.sim_get_position()
            x_actual = pos[0]
            y_actual = pos[1]
            x, y, theta = self.pf.get_estimate()

        x, y, theta = self.pf.get_estimate()
        self.odometry.x = x
        self.odometry.y = y
        self.odometry.theta = theta

        # Reset Encoders
        self.create._leftEncoderCount = 0.0
        self.create._rightEncoderCount = 0.0



    def find_path(self):
        print('Finding Path\n')
        start = (self.odometry.x*100, self.map.height - (self.odometry.y*100))
        self.rrt.build(start, 3000, 10)
        x_goal = self.rrt.nearest_neighbor((40, 120))
        self.path = self.rrt.shortest_path(x_goal)

        for v in self.rrt.T:
            for u in v.neighbors:
                self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0,0,0))
        for idx in range(0, len(self.path)-1):
            self.map.draw_line((self.path[idx].state[0], self.path[idx].state[1]), (self.path[idx+1].state[0], self.path[idx+1].state[1]), (0,255,0))

        self.map.save("finalproject_rrt.png")

    def follow_path(self):
        print('Following Path\n')
        base_speed = 100
        print('Points in path: ')
        for p in self.path:
            goal_x = p.state[0] / 100.0
            goal_y =  (self.map.height - p.state[1]) / 100.0
            print(goal_x, goal_y)

        #self.create.drive_direct(-100, -100)
        #self.sleep(2)
        #self.create.drive_direct(0,0)

        for p in self.path:
            goal_x = (p.state[0] / 100.0)
            goal_y = (self.map.height - p.state[1]) / 100.0

            while True:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    output_theta = self.pidTheta2.update(self.odometry.theta, goal_theta, self.time.time())
                    self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
                    # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    if distance < 0.05:
                        break

    def run(self):
        self.create.start()
        self.create.safe()

        self.create.drive_direct(0, 0)

        self.arm.open_gripper()

        self.time.sleep(4)

        #self.arm.close_gripper()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])


        self.arm.go_to(4, math.radians(-90))
        self.time.sleep(4)
        self.visualize()


        self.localize_self()
        self.find_path()
        self.follow_path()



        #posC = self.create.sim_get_position()

        #print(posC)

        #self.arm.go_to(4, math.radians(-90))
        #self.arm.go_to(5, math.radians(90))
        #self.time.sleep(100)


        #self.time.sleep(0.01)
