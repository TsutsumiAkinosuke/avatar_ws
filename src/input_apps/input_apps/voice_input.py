import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr

class VoiceInputNode(Node):

    def __init__(self):

        super().__init__("voice_input_node")

        # 音声認識のインスタンス
        self.recognizer = sr.Recognizer()

        # 発声を終えてから録音を中断するまでの待機時間
        self.recognizer.pause_threshold = 0.5

        # マイクのインスタンス
        self.microphone = sr.Microphone()

        # 音声テキストのパブリッシャーを宣言
        self.voice_text_publisher = self.create_publisher(String, "/voice_text", 10)

        self.voice_text = String()
    
    def listen_speech(self):

        with self.microphone as source:
                
            while rclpy.ok():

                # ノイズ対策調整(1秒?)
                self.recognizer.adjust_for_ambient_noise(source)

                self.get_logger().info("Listening...")

                # 録音
                self.audio = self.recognizer.listen(source)

                try:
                    # GoogleのAPIを用いて音声認識
                    self.voice_text.data = self.recognizer.recognize_google(self.audio, language="ja-JP")
                    self.get_logger().info(f"You said: {self.voice_text.data}")
                    self.voice_text_publisher.publish(self.voice_text)

                except sr.UnknownValueError:
                    # 音声が理解できなかった場合
                    self.get_logger().info("Could not understand the audio.")

                except sr.RequestError as e:
                    # APIへのリクエストでエラーが発生した場合
                    self.get_logger().info(f"Could not request; {e}")

def main(args=None):

    rclpy.init()
    node = VoiceInputNode()

    try:
        node.listen_speech()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()