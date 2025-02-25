#!/usr/bin/env python3
# encoding: utf-8
# 自检程序(self-test program)
import os
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  
from ros_robot_controller_msgs.msg import BuzzerState, OLEDState


class BringUpNotifyNode(Node):
    def __init__(self):
        super().__init__('bringup_notify')
        # self.sdk = None
        self.ret = False
        self.lang = os.environ.get('ASR_LANGUAGE', 'English')
        
        
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        # data = os.popen('ls /dev/ |grep ring_mic').read()
        # if data == 'ring_mic\n':
            # os.system("ros2 launch xf_mic_asr_offline mic_init.launch.py enable_setting:=enable ")
        # 等待各个服务
        self.wait_for_services()

    def wait_for_services(self):
        self.get_logger().info("Waiting for services...")
        try:
            # 等待服务可用
            self.wait_for_service('/finger_trace/enter')
            self.wait_for_service('/object_sortting/enter')
            self.wait_for_service('/object_tracking/enter')
            self.wait_for_service('/tag_stackup/enter')
            self.wait_for_service('/waste_classification/enter')
            time.sleep(15) 
            # 如果所有服务都准备好了，设置为True
            self.ret = True
        except Exception as e:
            self.get_logger().error(f"Error waiting for services: {e}")

    def wait_for_service(self, service_name):
        client = self.create_client(Trigger, service_name)
        # while not client.wait_for_service(timeout_sec=4.0):  # 设置超时时间
            # self.get_logger().info('\033[1;33m%s\033[0m' % f"Service {service_name} not available, retrying...")
        # self.get_logger().info('\033[1;32m%s\033[0m' % f"Service {service_name} is now available.")

    def run(self):
        if self.ret:
              
            os.system('amixer -q -D pulse set Master {}%'.format(100))
            os.environ['AUDIODRIVER'] = 'alsa'

            if self.lang == 'Chinese':
                os.system('aplay -q -fS16_LE -r16000 -c1 -N --buffer-size=81920 ' + "/home/ubuntu/ros2_ws/src/bringup/voice/Chinese/running.wav")
                self.get_logger().info('\033[1;32m%s\033[0m' % "准备就绪")
            else:
                os.system('aplay -q -fS16_LE -r16000 -c1 -N --buffer-size=81920 ' + "/home/ubuntu/ros2_ws/src/bringup/voice/running.wav")
                self.get_logger().info('\033[1;32m%s\033[0m' % "I am ready")
            # 设置蜂鸣器和播放音频
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.2
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
            time.sleep(0.5)
        else:
            # 如果服务没有准备好，发出报警信号
            while rclpy.ok():
                msg = BuzzerState()
                msg.freq = 2500
                msg.on_time = 0.2
                msg.off_time = 0.01
                msg.repeat = 5
                self.buzzer_pub.publish(msg)
                time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = BringUpNotifyNode()
    
    # 在ROS2中使用spin来保持节点运行
    node.run()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
