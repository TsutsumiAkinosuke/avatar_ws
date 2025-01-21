import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from avatar_interfaces.action import MoveArmToMarker
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from action_msgs.msg import GoalStatus

import cv2
import numpy as np
from pymycobot import MyCobot
import time
from collections import deque

class PickupManagerNode(Node):

    def __init__(self):

        super().__init__("pickup_manager_node")

        self.image_subscriber = self.create_subscription(Image, "image_raw", self.image_callback, 10)

        #self.image_publisher = self.create_publisher(Image, "image_raw/marker", 10)

        self.twist_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.twist_msg = Twist()

        self.bridge = CvBridge()

        #-------- myCobot -------- #

        # myCobot制御モジュール
        self.mc = MyCobot("/dev/ttyACM0", 115200)

        # シーケンス番号
        self.sequence_num = 0

        # False:アームを動かさない(指令待ち)
        self.arm_flag = False

        # アームの関節角度
        self.angles = [0.0] * 6

        # アームの移動速度
        self.arm_speed = 10

        # アーム待機姿勢の角度
        self.idle_angles = [0, -30, -120, 140, 0, 45]

        self.close_count = 0
        self.sequence_num = 0

        # アイドル状態フラグ
        self.is_idle = False

        # アームを待機姿勢に移行
        self.arm_idle()

        #-------- ArUco -------- #

        # ArUcoマーカー認識モジュール
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

        # カメラ行列
        self.camera_matrix = np.array([[530, 0, 340],
                                [0, 530, 230],
                                [0,   0,   1]], dtype=float)
        
        # 歪み係数
        self.dist_coeffs = np.zeros((4, 1), dtype=float)

        # マーカーサイズ(cm?)
        self.object_points = np.array([
            [-1.5, -1.5, 0.0],  # 左下
            [1.5, -1.5, 0.0],   # 右下
            [1.5, 1.5, 0.0],    # 右上
            [-1.5, 1.5, 0.0]    # 左上
        ], dtype=np.float32)

        # グリッパーを閉じるマーカー縦幅のしきい値
        self.marker_threshold = 50

        self.height_que = deque([0]*10, 10)

        #-------- Action -------- #

        # Actionサーバー
        self.action_server = ActionServer(self,
                                          MoveArmToMarker,
                                          "move_arm_to_marker",
                                          execute_callback=self.execute_callback,
                                          goal_callback=self.goal_callback,
                                          handle_accepted_callback=self.handle_accepted_callback,
                                          cancel_callback=self.cancel_callback)

        # ActionのFeedback用メッセージ
        self.feedback_msg = MoveArmToMarker.Feedback()
    
    # カメラ画像に対するコールバック
    def image_callback(self, msg):

        # 画像をImageメッセージからcv2形式へ変換
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 画像サイズを取得
        self.image_height = msg.height
        self.image_width = msg.width
        
        # マーカー認識
        self.corners, self.ids, _ = self.detector.detectMarkers(frame)

        # マーカー描画
        #cv2.aruco.drawDetectedMarkers(frame, self.corners, self.ids)

        # マーカー描画画像をpublish
        #self.image_publisher.publish(self.bridge.cv2_to_imgmsg(frame))

    # ArUcoマーカーの縦幅を計算して返す
    def calc_marker_height(self, target_id):

        for i, corner in enumerate(self.corners):

            # もしターゲットとして与えられたIDなら
            if self.ids[i] == target_id:

                # 2D座標を取得
                image_points = np.array(corner[0], dtype=np.float32)

                # マーカーの位置と回転を推定
                ret, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs)

                if ret:
                    # 回転ベクトルrvecを回転行列に変換
                    rotation_matrix, _ = cv2.Rodrigues(rvec)

                    # 回転行列からオイラー角を計算
                    sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
                    singular = sy < 1e-6
                    if not singular:
                        x_angle = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                        #y_angle = np.arctan2(-rotation_matrix[2, 0], sy)
                        #z_angle = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    else:
                        x_angle = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                        #y_angle = np.arctan2(-rotation_matrix[2, 0], sy)
                        #z_angle = 0

                    # オイラー角を度に変換
                    #x_angle = np.degrees(x_angle)
                    #y_angle = np.degrees(y_angle)
                    #z_angle = np.degrees(z_angle)

                    # マーカーの画像上の縦幅を計算(左右の平均を取る)
                    diff_y = np.mean([corner[0][3]-corner[0][0], corner[0][2]-corner[0][1]])

                    # 推定角度から縦幅を補正
                    marker_height = diff_y / np.cos(x_angle)

                    return marker_height
                
        return None
    
    def calc_marker_center(self, target_id):

        center_x = center_y = 0

        for i, corner in enumerate(self.corners):

            # もしターゲットとして与えられたIDなら
            if self.ids[i] == target_id:

                corner_points = corner[0]

                center_x = int(np.mean(corner_points[:, 0]))
                center_y = int(np.mean(corner_points[:, 1])*0.95)

                return [center_x, center_y]
        
        return None

    def arm_idle(self):

        # 待機姿勢を代入
        self.angles = self.idle_angles

        # myCobotに角度指令送信
        self.mc.send_angles(self.angles, self.arm_speed)

        # myCobotにグリッパー指令送信
        self.mc.set_gripper_value(15, 50)

    def arm_sequence(self):

        if self.arm_flag == True:
            
            # マーカー縦幅が25か30超えるまで：3/6で追従しながら前進
            # グリッパーしきい値：75~80?
            # [0, 0, -90, 90, 0, 45]

            # ターゲットのArUcoマーカーの中心座標を取得
            center = self.calc_marker_center(self.target_id)

            # 画像内にマーカーが認識できていれば
            if center:

                print(f"seq:{self.sequence_num}")

                # マーカーの縦幅を計算
                height = self.calc_marker_height(self.target_id)
                self.height_que.append(height)

                # 第3関節/第6関節：target_idのマーカーを追従
                self.angles[3] += (self.image_height/2-center[1])*0.015
                self.angles[0] += (self.image_width/2-center[0])*0.01

                if self.sequence_num == 0:

                    # 足回りでプレートに接近

                    self.mc.send_angles(self.angles, self.arm_speed)

                    # 縦幅が25以下なら前進
                    if height < 25:
                        self.twist_msg.linear.x = 0.05
                    else:
                        self.twist_msg.linear.x = 0.0
                    
                    # マーカーが画像端にあるなら回転
                    if center[0] < self.image_width*0.3:
                        self.twist_msg.angular.z = 0.05
                    elif center[0] > self.image_width*0.7:
                        self.twist_msg.angular.z = -0.05
                    else:
                        self.twist_msg.angular.z = 0.0

                    # 移動が不要なら次のシーケンスへ
                    if self.twist_msg.linear.x == 0.0 and self.twist_msg.angular.z == 0.0:
                        self.sequence_num = 1
                    
                    self.twist_publisher.publish(self.twist_msg)

                elif self.sequence_num == 1:

                    # アームを動かしてグリッパーをマーカーに近づける

                    # マーカーの縦幅がしきい値より小さければ
                    if np.mean(self.height_que) < self.marker_threshold:

                        # 第4関節/第5関節：関節の床からのを維持したままグリッパーを近づける
                        if self.angles[3] < 105:
                            self.angles[2] = min(0, self.angles[2]+1)
                            self.angles[1] = max(-90, self.angles[1]-1)

                        self.mc.send_angles(self.angles, self.arm_speed)

                        # Feedbackを送信
                        self.feedback_msg.message = "Approaching to the marker..."
                        self.goal_handle.publish_feedback(self.feedback_msg)
                    
                    # マーカー縦幅がしきい値より大きければ次のシーケンスへ
                    else:
                        self.sequence_num = 2
                
                elif self.sequence_num == 2:

                    # グリッパーを閉じて待機

                    if self.close_count < 20:
                    
                        # myCobotにグリッパー指令送信
                        self.mc.set_gripper_value(15, 100)

                        self.close_count += 1

                    # グリッパーが閉じたら次のシーケンスへ
                    else:
                        self.sequence_num = 3

                elif self.sequence_num == 3:

                    # 第2関節/第3関節の角度を維持したまま初期姿勢
                    self.mc.send_angles([self.idle_angles[0], self.idle_angles[1], self.idle_angles[2], self.angles[3], self.angles[4], self.idle_angles[5]], 10)

                    # Feedbackを送信
                    self.feedback_msg.message = "Close the gripper!"
                    self.goal_handle.publish_feedback(self.feedback_msg)

                    self.goal_handle.execute()

            # 画像内にマーカーが認識できていなければ
            else:

                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
                self.twist_publisher.publish(self.twist_msg)
                
                # 第3関節が床を向きすぎるまで
                if self.angles[3] > 90:

                    # 2度ずつカメラを下に向ける
                    self.angles[3] = self.angles[3] - 1
                    self.mc.send_angles(self.angles, self.arm_speed)

                # マーカーが見つからない場合
                else:
                    self.get_logger().info("Markers not found!")

    # Actionリクエストに対するコールバック
    def execute_callback(self, goal_handle):

        # アーム制御を停止する
        self.arm_flag = False
        self.timer.cancel()

        # Result
        result_msg = MoveArmToMarker.Result()
        result_msg.success = True

        goal_handle.succeed()

        self.get_logger().info("Goal succeeded!")

        return result_msg

    def goal_callback(self, goal_request):

        self.get_logger().info("Received goal request")

        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle):

        # アームを動作中でなければ指示を受け入れ
        if self.arm_flag == False:

            # ゴールハンドルをメンバに落とす
            self.goal_handle = goal_handle

            # リクエストされたIDを取得
            self.target_id = goal_handle.request.marker_id
            self.get_logger().info(f"Received new target id: {self.target_id}")

            # myCobotにグリッパー指令送信
            self.mc.set_gripper_value(255, 100)

            # 初期化
            self.close_count = 0
            self.sequence_num = 0

            # アームを動作中に更新
            self.arm_flag = True

            # タイマー割り込みを設定
            self.timer = self.create_timer(0.1, self.timer_callback)

            self.get_logger().info("Deferring execution...")
        
        # アーム動作中ならリクエストを拒否
        else:

            goal_handle.abort()
            self.get_logger().info("Request aborted.")

    def cancel_callback(self, goal_handle):

        self.get_logger().info("Received cancel request.")

        self.arm_flag = False

        # タイマー割り込みを停止
        try:
            self.timer.cancel()
        except:
            pass

        return CancelResponse.ACCEPT

    def timer_callback(self):

        # アームを一定時間間隔で制御
        self.arm_sequence()

def main(args=None):

    rclpy.init(args=args)
    node = PickupManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()