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

class Mode(Enum):
    RRT      = 1
    Localize = 2


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
        self.mode = Mode.RRT
        self.path = []
        self.is_localized = False

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


    def go_to_arm(self, goal):
        start = self.create.sim_get_position()
       
        #Start Localizing
        dist = self.sonar.get_distance()
        self.pf.measure(dist, 0)
        self.visualize()                     
            
        
        #Initial RRT
        start = self.create.sim_get_position()
        current_position = (start[0], start[1])
        goal_x = goal[0] / 100.0
        goal_y =  (self.map.height - goal[1]) / 100.0
        self.odometry.x = start[0]
        self.odometry.y = start[1]

        self.find_path(start, goal)
        self.is_localized = False
        not_done = True
        base_speed = 100

        while not_done == True:
            #print(self.mode)

            if self.mode == Mode.RRT:
                if self.is_localized == False:
                    #for p in self.path:
                    for i in range(2):
                        g_x = (self.path[i].state[0] / 100.0)
                        g_y = (self.map.height - self.path[i].state[1]) / 100.0
                        old_x = self.odometry.x
                        old_y = self.odometry.y
                        old_theta = self.odometry.theta
                        while True:
                            state = self.create.update()
                            if state is not None:
                                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                                goal_theta = math.atan2(g_y - self.odometry.y, g_x - self.odometry.x)
                                theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                                output_theta = self.pidTheta2.update(self.odometry.theta, goal_theta, self.time.time())
                                self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))


                                d = math.sqrt(math.pow(g_x - self.odometry.x, 2) + math.pow(g_y - self.odometry.y, 2))
                                if d < 0.05:
                                    break
                        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)
                        

                    self.mode = Mode.Localize
                    start = self.create.sim_get_position()
                    current_position = (start[0], start[1])
                    self.odometry.x = start[0]
                    self.odometry.y = start[1]
                    self.map = rrt_map.Map("configuration_space.png")
                    self.rrt = rrt.RRT(self.map)
                    self.find_path(start, goal)

                else:
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

                                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                               
                                if distance < 0.05:
                                    break
                    not_done = False
                    self.create.drive_direct(0,0)

            else:
                if self.is_localized == False:
                    self.create.drive_direct(0,0)
                    robotPos = self.create.sim_get_position()
                   
                    dist = self.sonar.get_distance()
                    self.pf.measure(dist, 0)
                    self.visualize()                     
                    x, y, theta = self.pf.get_estimate()
                    
        
                    self.mode = Mode.RRT
                    start = self.create.sim_get_position()
                    current_position = (start[0], start[1])
                    self.odometry.x = start[0]
                    self.odometry.y = start[1]
                    self.map = rrt_map.Map("configuration_space.png")
                    self.rrt = rrt.RRT(self.map)
                    self.find_path(start, goal)

                    if math.sqrt(math.pow(robotPos[0] - x, 2) + math.pow(robotPos[1] - y, 2)) < 0.1:
                        self.is_localized = True
                        
                        x_est, y_est, theta = self.pf.get_estimate()
                        """
                        self.odometry.x = x
                        self.odometry.y = y
                        self.odometry.theta = theta
                        """
                        start = self.create.sim_get_position()
                        self.odometry.x = start[0]
                        self.odometry.y = start[1]
                        self.odometry.theta = theta
                        self.map = rrt_map.Map("configuration_space.png")
                        self.rrt = rrt.RRT(self.map)
                        self.find_path(start, goal)

                else:
                    self.mode = Mode.RRT
                    start = self.create.sim_get_position()
                    current_position = (start[0], start[1])
                    self.odometry.x = start[0]
                    self.odometry.y = start[1]
                    self.map = rrt_map.Map("configuration_space.png")
                    self.rrt = rrt.RRT(self.map)
                    self.find_path(start, goal)


    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)


    def find_path(self, start, goal):
        print('Finding Path\n')
        begin = (start[0]*100, self.map.height - (start[1]*100))
        self.rrt.build(begin, 3000, 10)
        x_goal = self.rrt.nearest_neighbor(goal)
        self.path = self.rrt.shortest_path(x_goal)

        for v in self.rrt.T:
            for u in v.neighbors:
                self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0,0,0))
        for idx in range(0, len(self.path)-1):
            self.map.draw_line((self.path[idx].state[0], self.path[idx].state[1]), (self.path[idx+1].state[0], self.path[idx+1].state[1]), (0,255,0))

        self.map.save("finalproject_rrt.png")
    
    def get_arm_position_in_pixels(self, position):
        
        # If arm on East or West
        if 0.0250 <= position[1] <= 3.0250:
            if position[0] < 0.0250:
                pass
            else:
                pass

        # If Arm is on North
        elif position[1] > 0.0250:
            p = (((position[0])*100), self.map.height - ((position[1] - 0.969) * 100))
            #print("Position = {}".format(position))
            #print("Returning {}".format(p))
            return p

        # If Arm is on South
        else:
            pass

    def forward_kinematics(self, theta1, theta2):
        self.arm.go_to(1, theta1)
        self.time.sleep(2)
        self.arm.go_to(3, theta2)
        self.time.sleep(2)
        L1 = 0.4 # estimated using V-REP (joint2 - joint4)
        L2 = 0.39 # estimated using V-REP (joint4 - joint6)
        z = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2) + 0.3105
        x = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
        #print("Go to {},{} deg, FK: [{},{},{}]".format(math.degrees(theta1), math.degrees(theta2), -x, 0, z))
        return((x, z))

    def inverse_kinematics(self, x_i, z_i):
        L1 = 0.4 # estimated using V-REP (joint2 - joint4)
        L2 = 0.39 # estimated using V-REP (joint4 - joint6)
        # Corrections for our coordinate system
        z = z_i - 0.3105
        x = -x_i
        # compute inverse kinematics
        r = math.sqrt(x*x + z*z)
        alpha = math.acos((L1*L1 + L2*L2 - r*r) / (2*L1*L2))
        theta2 = math.pi - alpha

        beta = math.acos((r*r + L1*L1 - L2*L2) / (2*L1*r))
        theta1 = math.atan2(x, z) - beta
        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            theta2 = math.pi + alpha
            theta1 = math.atan2(x, z) + beta
        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            print("Not possible")
            return

        self.arm.go_to(1, theta1)
        self.arm.go_to(3, theta2)
        print("Go to [{},{}], IK: [{} deg, {} deg]".format(x_i, z_i, math.degrees(theta1), math.degrees(theta2)))

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

        #self.visualize()
       
        # First Face Arm Towards Ground
        #self.arm.go_to(5, math.radians(90))
        #self.time.sleep(2)

    
        # Tell Create To Go to Arm
        # Note: Please change Arms Coordinate Position here when changing start position of arm
        arm_postion = (1.6001, 3.3999)
        arm_position_in_pixel_coordinates = self.get_arm_position_in_pixels(arm_postion)
        self.go_to_arm(arm_position_in_pixel_coordinates)
        self.time.sleep(5)
        
        
        
        self.go_to_angle(math.radians(-90))
        self.time.sleep(5)
        
        # Put Arm Down
        ang = 1

        while ang <= 60:
            self.arm.go_to(1, math.radians(ang))
            self.time.sleep(0.1)
            ang = ang + 1
        
        
        ang = 1

        while ang <= 55:
            self.arm.go_to(3, math.radians(ang))
            self.time.sleep(0.1)
            ang = ang + 1

        self.arm.close_gripper()
        self.time.sleep(5)

        ang = 69

        while ang >= 0:
            self.arm.go_to(1, math.radians(ang))
            self.time.sleep(0.1)
            ang = ang - 1
        
        self.time.sleep(10)
       
       


        #posC = self.create.sim_get_position()

        #print(posC)

        #self.arm.go_to(4, math.radians(-90))
        #self.arm.go_to(5, math.radians(90))
        #self.time.sleep(100)


        #self.time.sleep(0.01)
