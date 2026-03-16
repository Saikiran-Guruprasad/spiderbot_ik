#!/usr/bin/env python3

from math import atan2, acos, sqrt, sin, cos, radians, degrees, pi

import rclpy
from rclpy.node import Node
from jetson_stm32_poc_msgs.msg import DataVector

#the class for ik ofc
class HexLeg:
    def __init__(self, l1, lc, l2, l3, ang_deg, neutral_angles=(0, 25, -75)):
        #initialisation of ik parameters
        self.l1  = l1
        self.lc  = lc
        self.l2  = l2
        self.l3  = l3
        self.ang = radians(ang_deg)

        #essentially used to find the initial x, y and z positions initially (at rest)
        _t2n, _t3n = neutral_angles[1] - 90, -neutral_angles[2]
        _xp0 = l2*cos(radians(_t2n)) + l3*cos(radians(_t2n + _t3n))
        _z0  = l2*sin(radians(_t2n)) + l3*sin(radians(_t2n + _t3n))

        self.x0 = (l1 + lc)*cos(self.ang) + _xp0*cos(self.ang)
        self.y0 = (l1 + lc)*sin(self.ang) + _xp0*sin(self.ang)
        self.z0 = _z0
        self.x  = self.x0
        self.y  = self.y0
        self.z  = self.z0

    #coxa function gives the x and y coordinates of servo 2 wrt origin of the bot
    def _coxa(self, t1):
        cx = self.l1*cos(self.ang) + self.lc*cos(self.ang + radians(t1))
        cy = self.l1*sin(self.ang) + self.lc*sin(self.ang + radians(t1))
        return cx, cy

    #theta 1, theta 2 and theta 3 using IK equations from IEEE document
    def theta_1(self, x, y):
        angle = atan2(y, x) - self.ang
        angle = (angle + pi) % (2*pi) - pi
        return degrees(angle)

    def theta_3(self, x, y, z):
        t1 = self.theta_1(x, y)
        cx, cy = self._coxa(t1)
        xp = sqrt((x - cx)**2 + (y - cy)**2)
        c = (xp*xp + z*z - self.l2*self.l2 - self.l3*self.l3) / (2*self.l2*self.l3)
        c = max(-1, min(1, c))
        return degrees(acos(c))

    def theta_2(self, x, y, z, t3):
        t1 = self.theta_1(x, y)
        cx, cy = self._coxa(t1)
        xp = sqrt((x - cx)**2 + (y - cy)**2)
        return degrees(atan2(z, xp) - atan2(self.l3*sin(radians(t3)),
                                            self.l2 + self.l3*cos(radians(t3))))

    #rounded for easier debugging and angles changed as the parameters of angles given by IK eqns slightly differ, but with appropriate changes the angles become appropriate
    def ik(self):
        x, y, z = self.x, self.y, self.z
        t1 = self.theta_1(x, y)
        t3 = self.theta_3(x, y, z)
        t2 = self.theta_2(x, y, z, t3)
        return [round(t1, 3), round(t2 + 90, 3), round(-t3, 3)]

    #class solely dedicated to rotation operation of the bot (rotates in same place)
class Rotation:
    def __init__(self, x, y, z):            #z usage can be tied to delays, later implement
        self.final_angle = -degrees(atan2(x, y))
        self.B = 5 if self.final_angle > 0 else -5 #arbitrary degrees chosen
        self.no_of_cycles = round(abs(self.final_angle/(2*self.B)),0)
        self.phase = 0
        self.no_of_cycles_completed = 0
        self.step_size = (10/z) + 10
        

#these bottom 2 functions bezier and cal work, so that the curve of the bezier can be traced by the legs of the bot
# STEPS CAN BE CHANGED ACCORDING TO REQUIREMENTS
def bezier(t, P1, P2, P3):
    y = (1-t)**2 * P1[0] + 2*(1-t)*t * P2[0] + t**2 * P3[0]
    z = (1-t)**2 * P1[1] + 2*(1-t)*t * P2[1] + t**2 * P3[1]
    return y, z

def cal(t, leg, flag, lift=2.0, steps=20):
    P1 = (leg.y0 - 1, leg.z0)
    P2 = (leg.y0,leg.z0 + lift)
    P3 = (leg.y0 + 1, leg.z0)

    if flag == 1:
        t += 1/steps
        t = min(t, 1.0) 
        leg.y, leg.z = bezier(t, P1, P2, P3)
        if t >= 1.0:
            flag = -1
            t = 0.0

    elif flag == -1:
        leg.y -= 1/steps
        leg.z  = leg.z0
        if (leg.y - leg.y0) <= -1.0:
            leg.y = leg.y0 - 1.0
            flag  = 1
            t = 0.0

    return t, flag


#Final ROS2 node for operation
class FinalIKNode(Node):
    def __init__(self):
        super().__init__('final_ik_node')

        #init values for forward motion
        self.buffer = None
        self.num = DataVector()
        self.num.position_x = 0.0
        self.num.position_y = 0.0
        self.num.twist_z = 0.0
        self.is_cycle_complete = True
        self.step_count = 0

        #init values for rotation
        self.rotation_trigger = False
        self.rotation_complete = True

        #init values for delay function
        self.delay_clicks = 0
        self.delayer = 0
        self.delay_number = 0

        # add to __init__:
        self.legs     = None
        self.t_params = [0.5, 0.0, 0.5, 0.0, 0.5, 0.0]
        self.flags    = [1, -1, 1, -1, 1, -1]
        self.output   = [[0, 25, -75]] * 6
        self.output0  = [[0, 25, -75]] * 6
        
        self.reader = self.create_subscription(
                DataVector,
                'sender_data',
                self.read_data_buffer,
                10)
        self.get_logger().info('Final IK Node started, waiting for commands...')
        self.timer = self.create_timer(0.02, self.read_data)
    
    #buffer kept to check whether the data given by user matches the data being run before command is given
    def read_data_buffer(self, msg):
        self.buffer = msg
    
    # MAIN FUNCTION OF THE PROGRAM, SINCE IT IS CALL-BACKED EVERY 0.02 SECONDS IT CONTAINS THE MAIN LOGIC FOR GAIT, ROTATION AND DELAY MOVEMENTS
    def read_data(self):
        if self.delayer == 1:
            self.delay()
            return
        if self.buffer is None:
            return

        if (self.buffer.position_x != self.num.position_x or self.buffer.position_y != self.num.position_y or self.buffer.twist_z != self.num.twist_z):
            if self.is_cycle_complete or (self.num.position_x==0.0 and self.num.position_y==0.0 and self.num.twist_z==0.0) or self.rotation_complete:

                if self.buffer.position_y==self.num.position_y and self.buffer.position_x==0.0 or self.buffer.position_x==self.num.position_x and self.buffer.position_y==0.0:
                    pass
                else:
                    self.output = [[0,25,-75]]*6
                    for i in self.output:
                        self.get_logger().info(f"{i[0]}   {i[1]}   {i[2]}")
                    self.get_logger().info('\n\n\n')
                    if self.delayer == 0:
                        self.delay(11)
                        return
                    elif self.delayer == -1:
                        self.delayer = 0

                
                self.num.position_x = self.buffer.position_x
                self.num.position_y = self.buffer.position_y
                self.num.twist_z    = self.buffer.twist_z
                self.is_cycle_complete = False

                if self.buffer.twist_z != 0.0:
                    self.rotation_trigger = True
                    self.rotation_complete = False
                    self.output = [[0, 27.192, -83.301], [0, 25, -75], [0, 27.192, -83.301], [0, 25, -75], [0, 27.192, -83.301], [0, 25, -75]]
                    for i in self.output:
                        self.get_logger().info(f"{i[0]}   {i[1]}   {i[2]}")
                    self.get_logger().info('\n\n\n')
                    self.rotator = Rotation(self.buffer.position_x, self.buffer.position_y, self.buffer.twist_z)
                    if self.delayer == 0:
                        self.delay(11)
                        return
                    elif self.delayer == -1:
                        self.delayer = 0   #this delay sets self.delayer to -1, but doesnt change it to 0 as it doesnt re-enter the loop
                    return

                else:
                    self.rotation_complete = False
                    self.rotation_trigger = False
                    self.get_logger().info(f"GAIT CHANGE")
                    self.transverse(self.buffer.position_x, self.buffer.position_y)

        if self.rotation_trigger == True:
            if self.rotator.phase == 0:
                for i in [1,3,5]:
                    self.output[i] = [self.rotator.B, 25, -75]
                if self.delayer == 0:
                    self.delay(11)
                    return
                elif self.delayer == -1:
                    self.delayer = 0
            elif self.rotator.phase == 1:
                for i in [0,2,4]:
                    self.output[i] = [0, 25, -75]
                if self.delayer == 0:
                    self.delay(11)
                    return
                elif self.delayer == -1:
                    self.delayer = 0
            elif self.rotator.phase == 2:
                for i in [1,3,5]:
                    self.output[i] = [0, 27.192, -83.301]
                if self.delayer == 0:
                    self.delay(11)
                    return
                elif self.delayer == -1:
                    self.delayer = 0
            elif self.rotator.phase == 3:
                for i in [0,2,4]:
                    self.output[i] = [self.rotator.B, 25, -75]
                if self.delayer == 0:
                    self.delay(11)
                    return
                elif self.delayer == -1:
                    self.delayer = 0
            elif self.rotator.phase == 4:
                for i in [1,3,5]:
                    self.output[i] = [0, 25, -75]
                if self.delayer == 0:
                    self.delay(11)
                    return
                elif self.delayer == -1:
                    self.delayer = 0
            elif self.rotator.phase == 5:
                for i in [0,2,4]:
                    self.output[i] = [0, 27.192, -83.301]
                if self.delayer == 0:
                    self.delay(11)
                    return
                elif self.delayer == -1:
                    self.delayer = 0
            elif self.rotator.phase == 6:
                self.rotator.no_of_cycles_completed += 1

                if (self.rotator.no_of_cycles_completed >= self.rotator.no_of_cycles):
                    if self.delayer != -1:
                        self.output = [[0, 25, -75]]*6
                        for i in self.output:
                            self.get_logger().info(f"{i[0]}   {i[1]}   {i[2]}")
                        self.get_logger().info('\n\n\n')

                    self.rotation_trigger = False
                    self.rotation_complete = True
                    self.num.position_x = 0.0
                    self.num.position_y = 1.0
                    self.num.twist_z = 0.0
                    self.buffer.position_x = 0.0
                    self.buffer.position_y = 1.0
                    self.buffer.twist_z = 0.0
                    self.rotator.phase = 0 # was -1 changed to 0

                    self.transverse(self.num.position_x, self.num.position_y)
                    self.rotation_complete = False #added this

                    if self.delayer == 0:
                        self.delay(11)
                        return                                  #this delayer DOES somehow work even tho self.delayer is -1
                    elif self.delayer == -1:
                        self.delayer = 0

                elif (self.buffer.position_x != self.num.position_x or self.buffer.position_y != self.num.position_y or self.buffer.twist_z != self.num.twist_z) and self.buffer.twist_z == 0.0:
                    if self.delayer != -1:
                        self.output = [[0, 25, -75]]*6
                        for i in self.output:
                            self.get_logger().info(f"{i[0]}   {i[1]}   {i[2]}")
                        self.get_logger().info('\n\n\n')

                    self.rotation_trigger = False
                    self.rotation_complete = True
                    self.num.position_x = self.buffer.position_x
                    self.num.position_y = self.buffer.position_y
                    self.num.twist_z = 0.0
                    self.rotator.phase = 0 # was -1 changed to 0

                    self.transverse(self.num.position_x, self.num.position_y)
                    self.rotation_complete = False #added this

                    if self.delayer == 0:
                        self.delay(11)
                        return                                  
                    elif self.delayer == -1:
                        self.delayer = 0
                
                else:
                    self.rotator.phase = -1
                    return  

    

            for i in self.output:
                self.get_logger().info(f"{i[0]}   {i[1]}   {i[2]}")
            self.get_logger().info('\n\n\n')
            self.rotator.phase += 1
            return

        if self.num != None and (self.num.position_x!=0.0 or self.num.position_y!=0.0 or self.num.twist_z!=0.0):
            self.is_cycle_complete = False
            for i in range(6):
                self.t_params[i], self.flags[i] = cal(self.t_params[i], self.legs[i], self.flags[i])
                self.output[i] = self.legs[i].ik()
            for i in self.output:
                self.get_logger().info(f"{i[0]}   {i[1]}   {i[2]}")
            self.get_logger().info('\n\n\n')
            self.step_count += 1
            if self.step_count%60 == 0:
                self.is_cycle_complete = True
                self.step_count = 0

    # function to initialise the 6 leg objects from hexaleg.
    def transverse(self, x_pos, y_pos):
        if x_pos == 0.0 and y_pos == 0.0:
            self.output = [[0, 25, -75]] * 6
            return
        self.step_count = 0
        B = -degrees(atan2(x_pos, y_pos))
        angles   = [60-B, 0-B, -60-B, -120-B, 180-B, 120-B]
        self.legs     = [HexLeg(l1=8, lc=2, l2=7, l3=5, ang_deg=a) for a in angles]
        self.output   = [[0, 0, 0]] * 6
        self.t_params = [0.5, 0.0, 0.5, 0.0, 0.5, 0.0]
        self.flags    = [1, -1, 1, -1, 1, -1]
        for i in range(6):
            if self.flags[i] == 1:
                self.legs[i].y, self.legs[i].z = bezier(0.5,
                                        (self.legs[i].y0 - 1, self.legs[i].z0),
                                        (self.legs[i].y0,     self.legs[i].z0 + 2.0),
                                        (self.legs[i].y0 + 1, self.legs[i].z0))
            else:
                self.legs[i].y = self.legs[i].y0
                self.legs[i].z = self.legs[i].z0
        self.output0 = [self.legs[i].ik() for i in range(6)]

    # delayer logic to give sufficient time for servos to attain sudden change in position that doesnt require small angle changes
    def delay(self, clicks=0):
        self.delayer = 1
        if clicks != 0:
            self.delay_number = clicks
        if self.delay_clicks == self.delay_number:
            self.delayer = -1
            self.delay_clicks = 0
            return
        self.delay_clicks += 1
        return


def main(args=None):
    rclpy.init(args=args)
    node = FinalIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()