#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                     
from rclpy.node   import Node         
from std_msgs.msg import UInt8                  
from std_msgs.msg import String
from sensor_msgs.msg import JointState                 
from rboot_libs import rbootlibs
from rboot_interface.msg import RbootPosition
import struct
import math

class SubscriberNode(Node):
    
    def __init__(self, name):
        super().__init__(name)
        self.previous_data = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.tolerance = 0.001
        self.client = rbootlibs.UDPClient('192.168.8.88', 9999)
        self.client.connect()
        self.declare_parameter('enable_status', 0)
        self.pub = self.create_publisher(RbootPosition, "/controller_status", 10) 
        self.sub = self.create_subscription(\
            RbootPosition, "/position", self.send_position, 10)        
        self.sub = self.create_subscription(\
            UInt8, "/loop", self.set_loop, 10)    
           
        self.sub = self.create_subscription(\
            JointState, "/joint_states", self.set_rviz_pos, 10)       
        
        enable_status = self.get_parameter('enable_status').get_parameter_value().integer_value
        self.get_logger().info('enable status is %d' %enable_status)
        if enable_status == 1:
            self.client.start_receive_thread()
            self.register_cb()
        else:
            self.unregister_cb()            

    def udp_callback(self, data):
         self.update(data)

    def send_msg(self, id, type, cmd1, cmd2) -> None:
         self.client.send_message(id, type, struct.pack('<I', cmd1), struct.pack('<I', cmd2), rbootlibs.Message_type['short'])
    
    def register_cb(self):
        self.client.register_callback(self.udp_callback) 
        self.send_msg(1, rbootlibs.Command_id['Set_Axis_State'], rbootlibs.AxisState['IDLE'], rbootlibs.Message_type['short'])        
        self.get_logger().info('Rboot CAN BUS: enabled controller status updates')        

    def unregister_cb(self):
        self.client.unregister_callback() 
        self.get_logger().info('Rboot CAN BUS: controller status is disabled')        
 
    def update(self, data):
        #data = client.get_buffer_data()
        if data is None:
            self.get_logger().info('Rboot CAN BUS: disabled as no more availabe data')        
        else:
            # Split the string by spaces and convert each hexadecimal value to an integer
            int_values = [int(x, 16) for x in data.split()]
            byte_array = struct.pack('12B', *int_values)
            if len(byte_array) == 12:
                msg = rbootlibs.pack_can_message(byte_array)
                id = msg.get('id')
                if id >= 0x30: self.get_logger().info('CAN BUS ID MUST less than 48(0x30)!!!')
                # for k,v in rbootlibs.motors_cfg.items():
                #     count += 1
                #     if k == 'M'+str(count) and count == id:
                type = msg.get('type')
                body = msg.get('body')
                if type == rbootlibs.Command_id['Get_Encoder_Estimates']:
                    pos, vel = struct.unpack('<ff', body)
                    # v['position'] = pos
                    # v['velocity'] = vel
                    pub_msg = RbootPosition()
                    pub_msg.nodeid = id                                     
                    pub_msg.position = pos                                 
                    pub_msg.velocity = vel
                    pub_msg.torque = 1.0
                    self.pub.publish(pub_msg)
                # elif type == rbootlibs.Command_id['Heartbeat']:
                #     error, state, result, traj_done, reserved = struct.unpack('<IBBBB', body)
                #     if result == 0:
                #         # v['status'] = state
                #         # v['error'] = error
                # elif type == rbootlibs.Command_id['Get_Bus_Voltage_Current']:
                #         vol, iq = struct.unpack('<ff', body)
                #         # v['voltage'] = round(vol, 2)
                #         # v['ibus'] = iq
                # elif type == rbootlibs.Command_id['Get_Iq']:
                #         iqs, iq = struct.unpack('<ff', body)
                #     #  v['ibus'] = iqs
                #         v['iq'] = iq 
                # elif type == rbootlibs.Command_id['Get_Temperature']:
                #         f, m = struct.unpack('<ff', body)
                #         print(f, m)

    def get_reduction(self, i):
        motor_keys = list(rbootlibs.motors_cfg.keys()) 
        motor_name = motor_keys[i]
        return rbootlibs.motors_cfg[motor_name]['reduction']

    def get_ccw(self, i):
        motor_keys = list(rbootlibs.motors_cfg.keys()) 
        motor_name = motor_keys[i]
        return rbootlibs.motors_cfg[motor_name]['ccw']

    def toggle_sign(self, x):
        if x > 0:
            return -x  
        elif x < 0:
            return abs(x)  
        else:
            return x  
    
    def send_6d_msg(self, id, type, cmd1, cmd2):
        for i in range(6):
            cid = i + 1
            self.client.send_message(cid, type, struct.pack('<I', cmd1), struct.pack('<I', cmd2), rbootlibs.Message_type['short'])

    def set_loop(self, msg):
        if msg.data == 1:
            self.send_6d_msg(0, rbootlibs.Command_id['Set_Axis_State'], rbootlibs.AxisState['CLOSED_LOOP_CONTROL'], 0)
            self.get_logger().info('loop state is set up! %d ' % (msg.data))
        else:
            self.send_6d_msg(0, rbootlibs.Command_id['Set_Axis_State'], rbootlibs.AxisState['IDLE'], 0)
            self.get_logger().info('loop state is clean up! %d ' % (msg.data))
 
    def set_rviz_pos(self, msg):
        for i, n in enumerate(msg.name):
            last_c = n[-1]
            if last_c.isdigit():
                joint_id = int(last_c)
                p = msg.position[i]

                difference = abs(p - self.previous_data[joint_id-1]) 
                if difference > self.tolerance:
                    self.previous_data[joint_id-1] = p
                    # self.get_logger().info('got rviz publishing data %f and previous %f' % (p, self.previous_data[i]))
                    reduction_value = self.get_reduction(joint_id-1)
                    motorCnt = (p * (180 / math.pi))  / 360.0 * reduction_value
                    self.get_logger().info('node %d and pos %f cnt %f' % (joint_id, p, (360.0 * motorCnt)/ reduction_value))
                    ccw = self.get_ccw(joint_id-1)
                    if ccw == 1:
                        p1 = motorCnt * 1
                    else:
                        p1 = motorCnt * -1
                    pos = struct.pack('<f', float(p1))
                    cmd2 = struct.pack('<HH', 60, 10)
                    # cmd2 = struct.pack('<HH', 0x1f, 0)
                    self.client.send_message(joint_id, rbootlibs.Command_id['Set_Input_Pos'], pos, cmd2, rbootlibs.Message_type['short'])

    def send_position(self, msg):
        reduction_value = self.get_reduction(msg.nodeid - 1)
        motorCnt = msg.position / 360.0 * reduction_value
        ccw = self.get_ccw(msg.nodeid - 1)
        if ccw == 1:
            p = self.toggle_sign(motorCnt)
        else:
            p = motorCnt
        pos = struct.pack('<f', float(p))
        cmd2 = struct.pack('<HH', 0x1f, 0)
        self.client.send_message(msg.nodeid, rbootlibs.Command_id['Set_Input_Pos'], pos, cmd2, rbootlibs.Message_type['short'])
        self.get_logger().info('sent node: %d position: %f degree: %f' % (msg.nodeid, msg.position, p))        
        
def main(args=None):                                
    rclpy.init(args=args)                            
    node = SubscriberNode("topic_rboot_sub")   
    rclpy.spin(node)                                 
    node.destroy_node()                              
    rclpy.shutdown() 

# ros2 topic pub --once /loop std_msgs/msg/UInt8 "{data: 1}"