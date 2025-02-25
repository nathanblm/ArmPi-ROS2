
#!/usr/bin/python3
# coding=utf8
# 通过深度图识别物体进行分类
# 机械臂向下识别
# 可以识别长方体，球，圆柱体，以及他们的颜色
import os
import cv2
import time
import math
import rclpy
import queue
import signal
import threading
import numpy as np
import message_filters
from rclpy.node import Node
from sdk import common, fps
from app.common import Heart
from cv_bridge import CvBridge
from interfaces.srv import SetStringList
from kinematics import kinematics_control
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics_msgs.srv import SetJointValue, SetRobotPose
from kinematics.kinematics_control import set_joint_value_target
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController
from example.rgbd_function.include.position_change_detect import position_reorder

def depth_pixel_to_camera(pixel_coords, intrinsic_matrix):
    fx, fy, cx, cy = intrinsic_matrix[0], intrinsic_matrix[4], intrinsic_matrix[2], intrinsic_matrix[5]
    px, py, pz = pixel_coords
    x = (px - cx) * pz / fx
    y = (py - cy) * pz / fy
    z = pz
    return np.array([x, y, z])


class ShapeRecognitionNode(Node):
    hand2cam_tf_matrix = [
        [0.0, 0.0, 1.0, -0.101],
        [-1.0, 0.0, 0.0, 0.011],
        [0.0, -1.0, 0.0, 0.045],
        [0.0, 0.0, 0.0, 1.0]
    ]
    pick_offset = [0.01, 0.01, 0.0, -0.01, 0.0]  # x1, x2, y1, y2, z
    '''
                
                 x1(+)
        y1(+)   center    y2(-)
                  x2

                  arm
    '''

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.fps = fps.FPS()
        self.moving = False
        self.stop = False
        self.count = 0
        self.running = True
        self.start = False
        self.recognition = False
        self.rgb = True
        self.img = True
        self.stop = False
        self.shapes = None
        self.image_sub = None
        self.depth_sub = None
        self.camera_info_sub = None
        self.target_shapes = ''
        self.roi = [60, 380, 160, 560]
        self.endpoint = None
        self.last_position = 0, 0
        self.last_object_info_list = []
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=2)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)

        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_srv = self.create_service(SetBool, '~/set_running', self.start_srv_callback)
        self.rgb_or_depth_srv = self.create_service(SetBool, '~/rgb_or_depth', self.rgb_or_depth_srv_callback)
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.create_service(SetStringList, '~/set_shape', self.set_shape_srv_callback)
        
        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)
        
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(SetBool, '/depth_cam/set_ldp_enable')
        self.client.wait_for_service()
       
        self.timer_cb_group = ReentrantCallbackGroup()
        self.set_joint_value_target_client = self.create_client(SetJointValue, '/kinematics/set_joint_value_target', callback_group=self.timer_cb_group)
        self.set_joint_value_target_client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/arm_pc/ActionGroups')

        Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)

        if self.get_parameter('start').value: 
            threading.Thread(target=self.enter_srv_callback,args=(Trigger.Request(), Trigger.Response()), daemon=True).start()
            req = SetBool.Request()
            req.data = True 
            res = SetBool.Response()
            self.start_srv_callback(req, res)

    def init_process(self):
        self.timer.cancel()
        msg = SetBool.Request()
        msg.data = False
        self.send_request(self.client, msg)
        self.goto_default()

        msg = SetStringList.Request()
        msg.data = ['sphere', 'cuboid', 'cylinder']
        self.set_shape_srv_callback(msg, SetStringList.Response())


        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "shutdown")

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def set_shape_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_shape")
        self.shapes = request.data
        self.start = True
        response.success = True
        response.message = "set_shape"
        return response


    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "loading shape recognition")
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)          
        self.rgb_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/depth_cam/depth/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')

        # 同步时间戳, 时间允许有误差在0.03s
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub], 3, 0.2)
        self.sync.registerCallback(self.multi_callback)
        self.stop = False
        self.start = True
        self.rgb = True
        self.img = True
        self.moving = False
        response.success = True
        response.message = "start"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "exit shape recognition")
        try:
            if self.rgb_sub is not None:

                self.destroy_subscription(self.rgb_sub)
                self.destroy_subscription(self.depth_sub)
                self.destroy_subscription(self.info_sub)   
                # if nc = None
                self.get_logger().info('\033[1;32m%s\033[0m' % "destroyed")
                self.image_sub = None
                self.depth_sub = None
                self.camera_info_sub = None
                self.img = False
        except Exception as e:
            self.get_logger().error(str(e))
        self.start = False
        self.stop = True
        self.shapes = None
        self.moving = True
        self.recognition = False
        self.count = 0
        self.target_shapes = ''
        self.last_position = 0, 0
        self.last_object_info_list = []
        response.success = True
        response.message = "stop"
        return response

    def start_srv_callback(self, request, response):
        if request.data:

            self.get_logger().info('\033[1;32m%s\033[0m' % "start shape recognition")
            self.start = True
            self.stop = False
            self.recognition = True
            response.success = True
            response.message = "start"
            return response
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % "stop shape recognition")
            # self.start = False
            self.stop = True
            self.recognition = False
            response.success = False
            response.message = "stop"
            return response

    def rgb_or_depth_srv_callback(self, request, response):
        if request.data:

            self.rgb = True
            response.success = True
            return response
        else:

            self.rgb = False
            response.success = False
            return response

    def go_home(self):
        set_servo_position(self.joints_pub, 1.0, ( (2, 560), (3, 130), (4, 115), (5, 500), (10, 200)))  
        time.sleep(1.0)
        set_servo_position(self.joints_pub, 1.0, ((1, 500), ))
        time.sleep(1.0)

    def goto_default(self):
        msg = set_joint_value_target([500, 520, 210, 50, 500])
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t = endpoint.pose.position
        pose_r = endpoint.pose.orientation
        set_servo_position(self.joints_pub, 1, ((2, 520), (3, 210), (4, 50), (5, 500)))
        time.sleep(1)

        set_servo_position(self.joints_pub, 1, ((1, 500),))
        time.sleep(1)

        set_servo_position(self.joints_pub, 0.5, ((10, 200),))
        time.sleep(0.5)

        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])

    def move(self, obejct_info):
        while True:
            shape, pose_t = obejct_info[:2]
            self.get_logger().info('\033[1;32m%s\033[0m' % pose_t)
            angle = obejct_info[-1]
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.2
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
            time.sleep(1)
            if 'sphere' in shape:
                offset_z = self.pick_offset[-1]
            elif 'cylinder' in shape:
                offset_z = 0.03 + self.pick_offset[-1]
            else:
                offset_z = 0.03 + self.pick_offset[-1]
            if pose_t[0] > 0.21:
                offset_x = self.pick_offset[0]
            else:
                offset_x = self.pick_offset[1]
            if pose_t[1] > 0:
                offset_y = self.pick_offset[2]
            else:
                offset_y = self.pick_offset[3]
            pose_t[0] += offset_x
            pose_t[1] += offset_y
            pose_t[2] += offset_z
            msg = kinematics_control.set_pose_target(pose_t, 85)
            res1 = self.send_request(self.kinematics_client, msg)
            if res1.pulse:
                servo_data = res1.pulse
                set_servo_position(self.joints_pub, 1.5, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, servo_data[4])))
                time.sleep(1.5)

            if self.stop:
                self.go_home()
                self.moving = False
                self.recognition = True
                break

            pose_t[2] -= 0.05
            msg = kinematics_control.set_pose_target(pose_t, 85)
            res2 = self.send_request(self.kinematics_client, msg)
            if angle != 0:
                if 'sphere' in shape or ('cylinder' in shape and 'cylinder_horizontal_' not in shape):
                    angle = 500
                else:
                    angle = angle % 180
                    angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)
                
                    angle = 500 + int(1000 * (angle + res2.rpy[-1]) / 240)
            else:
                angle = 500
            if res2.pulse:
                servo_data = res2.pulse
                set_servo_position(self.joints_pub, 0.5, ((5, angle),))
                time.sleep(0.5)
                set_servo_position(self.joints_pub, 1, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, angle)))
                time.sleep(1)
                set_servo_position(self.joints_pub, 0.6, ((10, 600),))
                time.sleep(0.6)

            if self.stop:
                self.go_home()
                self.moving = False
                self.recognition = True
                break

            if res1.pulse:
                servo_data = res1.pulse
                set_servo_position(self.joints_pub, 1, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, angle)))
                time.sleep(1)
            set_servo_position(self.joints_pub, 1, ((1, 500), (2, 520), (3, 220), (4, 50), (5, 500), (10, 650)))
            time.sleep(1)

            if self.stop:
                self.go_home()
                self.moving = False
                self.recognition = True
                break

            self.get_logger().info('shape: %s' % shape.split("_")[0])
            if not self.stop:
                self.get_logger().info('shape: %s' % shape.split("_")[0])
                if "sphere" in shape:
                    self.controller.run_action("target_1")
                if "cylinder" in shape:
                    self.controller.run_action("target_2")
                if "cuboid" in shape:
                    self.controller.run_action("target_3")
            self.goto_default()
            self.moving = False
            self.recognition = True
            break

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.img:
            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像
                self.image_queue.get()
            # 将图像放入队列
            self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def cal_position(self, x, y, depth, intrinsic_matrix):
        position = depth_pixel_to_camera([x, y, depth / 1000], intrinsic_matrix)
        position[0] -= 0.01
        pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))
        world_pose = np.matmul(self.endpoint, pose_end)
        pose_t, pose_r = common.mat_to_xyz_euler(world_pose)
        # self.get_logger().info('pose_t: %s, pose_r: %s' % (pose_t, pose_r))
        return pose_t

    def get_min_distance(self, depth_image):
        ih, iw = depth_image.shape[:2]
        # 屏蔽掉一些区域，降低识别条件，使识别跟可靠
        depth_image[:, :self.roi[2]] = np.array([[1000, ] * self.roi[2]] * ih)
        depth_image[:, self.roi[3]:] = np.array([[1000, ] * (iw - self.roi[3])] * ih)
        depth_image[self.roi[1]:, :] = np.array([[1000, ] * iw] * (ih - self.roi[1]))
        depth_image[:self.roi[0], :] = np.array([[1000, ] * iw] * self.roi[0])
        depth = np.copy(depth_image).reshape((-1,))
        depth[depth <= 0] = 55555  # 距离为0可能是进入死区，将距离赋一个大值

        min_index = np.argmin(depth)  # 距离最小的像素
        min_y = min_index // iw
        min_x = min_index - min_y * iw

        min_dist = depth_image[min_y, min_x]  # 获取距离摄像头最近的物体的距离
        return min_dist

    def get_contours(self, depth_image, min_dist):
        # self.get_logger().info('\033[1;32m%s\033[0m' % str(min_dist))
        depth_image = np.where(depth_image > 195, 0, depth_image)  
        depth_image = np.where(depth_image > min_dist + 20, 0, depth_image)  # 将深度值大于最小距离15mm的像素置0
        sim_depth_image_sort = np.clip(depth_image, 0, min_dist - 10).astype(np.float64) / (min_dist - 10) * 255
        depth_gray = sim_depth_image_sort.astype(np.uint8)
        _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours

    def shape_recognition(self, rgb_image, depth_image, depth_color_map, intrinsic_matrix, min_dist):
        object_info_list = []
        image_height, image_width = depth_image.shape[:2]
        # self.get_logger().info('\033[1;32m%s\033[0m' % str(min_dist))
        if min_dist <= 270:  # 机械臂初始位置并不是与桌面水平，大于这个值说明已经低于桌面了，可能检测有误，不进行识别
            sphere_index = 0
            cuboid_index = 0
            cylinder_index = 0
            cylinder_horizontal_index = 0
            contours = self.get_contours(depth_image, min_dist)
            
            for obj in contours:
                area = cv2.contourArea(obj)
                if area < 300:
                    continue
                perimeter = cv2.arcLength(obj, True)  # 计算轮廓周长
                approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)  # 获取轮廓角点坐标

                CornerNum = len(approx)
                (cx, cy), r = cv2.minEnclosingCircle(obj)
                center, (width, height), angle = cv2.minAreaRect(obj)
                if angle < -45:
                    angle += 89
                if width > height and width / height > 1.5:
                    angle = angle + 90
                depth = depth_image[int(cy), int(cx)]
                position = self.cal_position(cx, cy, depth, intrinsic_matrix)
                x, y, w, h = cv2.boundingRect(approx)
                mask = np.full((image_height, image_width), 0, dtype=np.uint8)
                
                cv2.drawContours(mask, [obj], -1, (255), cv2.FILLED)
                
                # 计算轮廓区域内像素值的标准差
                depth_image_mask = np.where(depth_image == 0, np.nan, depth_image)
                depth_std = np.nanstd(mask)
                objType = None
                if depth_std > 50.0 and CornerNum > 4:
                    sphere_index += 1
                    angle = 0
                    objType = 'sphere_' + str(sphere_index)
                elif depth_std > 50.0 :
                    cuboid_index += 1
                    objType = "cuboid_" + str(cuboid_index)

                elif depth_std > 40.0:
                    if abs(width/height - 1) < 0.2:
                        cuboid_index += 1
                        objType = "cuboid_" + str(cuboid_index)
                    else:
                        cylinder_horizontal_index += 1
                        objType = "cylinder_horizontal_" + str(cylinder_horizontal_index)

                if depth_std < 35.0 and CornerNum > 4:
                    cylinder_index += 1
                    angle = 0
                    objType = "cylinder_" + str(cylinder_index)
                if objType is not None:
                    object_info_list.append([objType, position, depth, [x, y, w, h, center, width, height], rgb_image[int(center[1]), int(center[0])], angle])
                    cv2.rectangle(depth_color_map, (x, y), (x + w, y + h), (255, 255, 255), 2)
        return object_info_list

    def main(self):
        count = 0
        while self.running:
            try:
                ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                depth_image = depth_image.copy()
                min_dist = self.get_min_distance(depth_image)
                
                #像素值限制在0到350的范围内, 将深度图像的像素值限制和归一化到0到255的范围内
                sim_depth_image = np.clip(depth_image, 0, 350).astype(np.float64) / 350 * 255
                    
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                if not self.moving and not self.stop:
                    object_info_list = self.shape_recognition(rgb_image, depth_image, depth_color_map, depth_camera_info.k, min_dist)
                    if self.start:
                        reorder_object_info_list = object_info_list
                        if object_info_list:
                            if self.last_object_info_list:
                                # 对比上一次的物体的位置来重新排序
                                reorder_object_info_list = position_reorder(object_info_list, self.last_object_info_list, 20)
                        if reorder_object_info_list:
                            if not self.target_shapes:
                                if self.shapes is not None:
                                    indices = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] in self.shapes]
                                if indices:
                                    min_depth_index = min(indices, key=lambda i: reorder_object_info_list[i][2])
                                    self.target_shapes = reorder_object_info_list[min_depth_index][0].split('_')[0]
                            else:

                                target_index = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] == self.target_shapes]
                                if target_index:
                                    target_index = target_index[0]
                                    obejct_info = reorder_object_info_list[target_index]
                                    x, y, w, h, center, width, height = obejct_info[3]
                                    angle = obejct_info[-1]
                                    cv2.putText(depth_color_map, self.target_shapes, (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                    (0, 0, 0), 2, cv2.LINE_AA)
                                    cv2.putText(depth_color_map, self.target_shapes, (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                    (255, 255, 255), 1)
                                    cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1,
                                                         (0, 0, 255), 2, cv2.LINE_AA)
                                    position = obejct_info[1]
                                    e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(
                                            pow(self.last_position[1] - position[1], 2)), 5)
                                    if self.recognition and e_distance <= 0.005:
                                        self.count += 1
                                    else:
                                        self.count = 0
                                    if self.count > 15:
                                        self.count = 0
                                        self.target_shapes = None
                                        self.recognition = False
                                        self.moving = True

                                        threading.Thread(target=self.move, args=(obejct_info,)).start()
                                    self.last_position = position
                                else:
                                    self.target_shapes = None

                        self.last_object_info_list = reorder_object_info_list

                bgr_image = cv2.cvtColor(rgb_image[40:440, ], cv2.COLOR_RGB2BGR)
                cv2.rectangle(bgr_image, (self.roi[2], self.roi[0]), (self.roi[3], self.roi[1]), (0, 255, 255), 1)

                if self.rgb:
                    self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))

                else:
                    self.result_publisher.publish(self.bridge.cv2_to_imgmsg(depth_color_map, "bgr8"))
                if bgr_image is not None and depth_color_map is not None and self.get_parameter('display').value:
                    result_image = np.concatenate([bgr_image, depth_color_map], axis=1)
                    cv2.imshow("depth", result_image)
                    key = cv2.waitKey(1)



            except Exception as e:
                self.get_logger().info(str(e))
        rclpy.shutdown()

def main():
    node = ShapeRecognitionNode('shape_recognition')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()

