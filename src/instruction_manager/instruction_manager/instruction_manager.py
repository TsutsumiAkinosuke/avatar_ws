import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import re

init_prompt = "You are a robot with a robotic arm.\
Create a code to accomplish the instructions given using the following functions and the coordinates on the map.\
The instructions are entered using voice recognition in Japanese, but please note that some characters may be mistranslated.\
Generate code using only the given functions and variables, and output only the code.\
For this input, please return a code that only speaks “起動しました” to try it out.\n\n\
\
Function\n\
move(x,y,q_w,q_z): Move the robot to the given x,y coordinates and face in the direction q_w,q_z.\n\
say(text): Read the text out loud.\n\n\
\
Coordinates and direction\n\
Shelf A: 7.17, 4.5, 0.707, 0.707\n\
Shelf B: -4, -1.4, 1, 0\n\
Shelf C: -4.5, 4.75, 0.707, 0.707\n\
Starting point: -0.6, 1.2, 0.707, 0.707"

class InstructionManagerNode(Node):

    def __init__(self):

        super().__init__("instruction_manager_node")

        self.voicetext_subscriber = self.create_subscription(String, "/voice_text", self.usertext_callback, 10)

        self.instruction = String()
        self.instruction_publisher = self.create_publisher(String, "/gemini/input_text", 10)
        self.response_subscriber = self.create_subscription(String, "/gemini/output_text", self.gemini_response_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # コード解析に利用する正規表現
        self.regular_pattern = r'(\w+)\((.*?)\)'

        # 何行目のコードを実行しているか
        self.sequence_index = 0
        self.pre_sequence_index = 0

        # コード解析の結果(各行ごと)を格納するリスト
        self.parse_results = []

        # NavigateToPose用メッセージ
        self.goal_msg = NavigateToPose.Goal()

        # ゴール姿勢を格納
        self.goal_pose = PoseStamped()

        # geminiが処理中か否かを示すフラグ
        self.instruction_flag = False

        # 初期プロンプトを代入
        self.instruction.data = init_prompt
        print(self.instruction.data)

        self.instruction_publisher.publish(self.instruction)
        self.instruction_flag = True

        self.get_logger().info("Waiting for code...")
    
    def usertext_callback(self, msg):

        # geminiが処理中でなければ指示を送信
        if self.instruction_flag == False:
        
            self.instruction.data = msg.data

            # 6文字以下の指示は無視
            if len(self.instruction.data) > 6:

                self.sequence_index = 0

                self.instruction_publisher.publish(self.instruction)

                self.instruction_flag = True

                self.get_logger().info(f"Instruction: {self.instruction.data}")
            
            else:
                self.get_logger().info(f"Received the text but not the instruction.")

        else:
            self.get_logger().info(f"Received the instruction but now processing another one.")

    def is_numeric_string(self, s):
        try:
            float(s)
            return True
        except ValueError:
            return False

    def code_check(self, func_name, args):

        if func_name == "move":
            # 引数が4つで, すべての引数が数値であれば問題なし
            return (len(args) == 4 and all(self.is_numeric_string(arg) for arg in args))

        elif func_name == "say":
            # 引数が1つで, その引数が文字列型であれば問題なし
            return (len(args) == 1 and type(args[0]) == str)
        
        return False

    # geminiからコードを受け取って解析
    def gemini_response_callback(self, msg):

        self.instruction_flag = False

        # コード解析結果を格納するリスト
        self.parse_results = []
        
        # コードを代入
        self.code = msg.data

        #self.code = "move(-4.0, -1.4, 0.0, 1.0)\nmove(-4.5, 4.75, 0.707, 0.707)"

        # 各行を解析
        try:
            for line in self.code.splitlines():

                match = re.search(self.regular_pattern, line)

                if match:

                    func_name = match.group(1)          # 関数名を抽出
                    args = match.group(2).split(", ")   # 引数をカンマ区切りで抽出

                    # 関数名と引数に誤りがないかチェック
                    if self.code_check(func_name, args):
                        self.parse_results.append((func_name, args))    # (関数名, 引数)のタプルをリストに追加
                        self.get_logger().info(f"{self.parse_results[-1]}")
                    else:
                        self.get_logger().info(f"Failed to code check {func_name}:{args}")
                        raise ValueError("")
            
            # 問題なく解析が完了したら1行目からコードを実行開始
            self.get_logger().info("Code successfully parsed!")
            self.sequence_index = 1

        except:

            # 解析に失敗
            self.get_logger().info("Failed to parse generated code...")
    
    # 一定時間おきに解析されたコードの実行確認
    def timer_callback(self):

        # シーケンス番号が0のとき(=全指示実行済み)またはシーケンス番号がコードの行数より大きいときはスキップ
        if self.sequence_index != 0 and self.sequence_index <= len(self.parse_results):

            # 実行するコードの行数が更新されたとき
            if self.sequence_index != self.pre_sequence_index:

                # 前ループのシーケンス番号を更新
                self.pre_sequence_index = self.sequence_index
                
                # 実行する処理がmove関数の場合
                if self.parse_results[self.sequence_index-1][0] == "move":
                    
                    # 引数からゴール姿勢を代入
                    self.goal_pose = PoseStamped()
                    self.goal_pose.header.frame_id = "map"
                    self.goal_pose.pose.position.x = float(self.parse_results[self.sequence_index-1][1][0])
                    self.goal_pose.pose.position.y = float(self.parse_results[self.sequence_index-1][1][1])
                    self.goal_pose.pose.orientation.z = float(self.parse_results[self.sequence_index-1][1][2])
                    self.goal_pose.pose.orientation.w = float(self.parse_results[self.sequence_index-1][1][3])

                    # メッセージにゴール姿勢を代入
                    self.goal_msg.pose = self.goal_pose

                    # ゴールをリクエスト
                    self.get_logger().info("Goal requesting...")
                    self.action_client.wait_for_server()
                    self.send_goal_future = self.action_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)

                    # ゴールリクエストに対するコールバックを登録
                    self.send_goal_future.add_done_callback(self.goal_response_callback)
            
                elif self.parse_results[self.sequence_index-1][0] == "say":
                    
                    # 発声内容を表示
                    self.get_logger().info(f"Say: {self.parse_results[self.sequence_index-1][1][0]}")

                    # 次のシーケンスに進める
                    self.sequence_index += 1
        else:

            self.sequence_index = 0
            self.pre_sequence_index = self.sequence_index

    # Actionのfeedback(ゴールに到達するまでに送られるフィードバック)に対するコールバック
    def feedback_callback(self, feedback_msg):
        
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback}')

    # Actionのresponse(ゴールリクエストに対して返されるレスポンス)に対するコールバック
    def goal_response_callback(self, future):

        # ゴールリクエストのレスポンスを取得
        goal_handle = future.result()

        # ゴールリクエストが却下されたら
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.parse_results = []
            self.sequence_index = 0
            self.pre_sequence_index = 0
            return

        # Actionのresult(結果)を取得
        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):

        # 結果を取得して表示
        result = future.result().result
        self.get_logger().info(f'Goal result: {result}')

        # 次のシーケンスに進める
        self.sequence_index += 1

def main(args=None):
    
    rclpy.init(args=args)
    node = InstructionManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()