import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import MoveArmToMarker  # MoveArmToMarkerアクション
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
import time

class ArUcoMarkerCenterNode(Node):
    def __init__(self):
        super().__init__('aruco_marker_center_node')

        # CV Bridge
        self.bridge = CvBridge()

        # 画像サブスクライブ
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',  # ここで使用する画像トピック名
            self.image_callback,
            10
        )

        # ArUcoマーカー辞書の選択
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # Actionサーバーの作成
        self.action_server = ActionServer(
            self,
            MoveArmToMarker,
            'move_arm_to_marker',
            self.execute_callback
        )

        # タイマーコールバック用変数
        self.timer = None
        self.target_position = None
        self.arm_moving = False
        self.goal_handle = None

    def image_callback(self, msg):
        # 画像メッセージをOpenCVの画像に変換
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return
        
        # ArUcoマーカーを検出
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)

        if ids is not None:
            for i, marker_id in enumerate(ids):
                marker_id = marker_id[0]  # IDの取得
                self.get_logger().info(f'Detected marker with ID: {marker_id}')

                # 特定のIDのマーカーの重心を求める
                if marker_id == 3:  # 例としてID 3のマーカーに対して重心を求める
                    centroid = self.calculate_centroid(corners[i])
                    self.get_logger().info(f'Centroid of marker {marker_id}: {centroid}')
        
        # マーカーを描画（オプション）
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        cv2.imshow('Detected ArUco Markers', cv_image)
        cv2.waitKey(1)

    def calculate_centroid(self, corners):
        # マーカーの4つの頂点 (四角形の中心を計算)
        corner_points = np.array(corners[0], dtype=np.float32)
        centroid = np.mean(corner_points, axis=0)
        return (centroid[0], centroid[1])

    def execute_callback(self, goal_handle):
        # アクションリクエストの処理
        marker_id = goal_handle.request.marker_id
        self.get_logger().info(f"Received request to move arm to marker with ID {marker_id}")

        # 対象のマーカーの座標を取得
        marker_position = self.get_marker_position(marker_id)
        
        # アームを指定位置に動かす（時間がかかる場合はtimer_callbackで定期的に状態を確認）
        self.target_position = marker_position
        self.arm_moving = True
        self.goal_handle = goal_handle

        # タイマーを使用して進捗をチェック
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1秒ごとに進捗確認

        # 初期の進捗フィードバック
        feedback_msg = MoveArmToMarker.Feedback()
        feedback_msg.progress = 0.0
        self.goal_handle.publish_feedback(feedback_msg)

        return 'Processing'

    def timer_callback(self):
        if self.arm_moving:
            # 仮のロジックで進捗を計算
            # 実際には、アームの位置や速度を確認して進捗を計算する
            self.get_logger().info(f"Moving arm to position {self.target_position}")
            progress = self.calculate_progress()

            # 進捗をフィードバックとして送信
            feedback_msg = MoveArmToMarker.Feedback()
            feedback_msg.progress = progress
            self.goal_handle.publish_feedback(feedback_msg)

            # アームが目的地に到達した場合
            if progress >= 100.0:
                self.timer.cancel()
                self.arm_moving = False
                self.return_result(True, "Arm successfully moved to target position.")

    def calculate_progress(self):
        # 仮の進捗計算。実際にはロボットアームの制御に基づいた進捗を返します。
        # ここでは進捗を簡単に100%で完了とします。
        return 100.0  # 仮の進捗 100%

    def get_marker_position(self, marker_id):
        # マーカーIDに基づいてマーカーの位置を決定
        # ここでは仮の位置を返しますが、実際の座標変換などが必要です。
        return (0.5, 0.2, 0.1)  # 例として仮の位置を返す

    def return_result(self, success, message):
        result = MoveArmToMarker.Result()
        result.success = success
        result.message = message
        if self.goal_handle:
            self.goal_handle.succeed()
            self.goal_handle.publish_result(result)

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoMarkerCenterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
