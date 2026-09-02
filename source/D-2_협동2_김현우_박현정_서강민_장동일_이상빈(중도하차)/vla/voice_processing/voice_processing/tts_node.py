import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import queue
import threading
import subprocess
from ament_index_python.packages import get_package_share_directory

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(String, '/say', self.say_callback, 10)
        self.msg_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self._process_queue, daemon=True)
        self.worker_thread.start()
        
        # 리소스 경로 설정
        package_path = get_package_share_directory('voice_processing')
        self.resource_path = os.path.join(package_path, 'resource')
        self.player = None
        self.get_logger().info("TTS Node ready.")

    def stop_audio(self):
        if self.player and self.player.poll() is None:
            self.player.terminate()
            self.player = None

    def say_callback(self, msg):
        if msg.data: self.msg_queue.put(msg.data)

    def _process_queue(self):
        while True:
            full_text = self.msg_queue.get()
            try:
                if full_text == "STOP_SOUND":
                    self.stop_audio()
                elif full_text == "BEEP::":
                    self.stop_audio()
                    self.player = subprocess.Popen(['ffplay', '-nodisp', '-autoexit', '-hide_banner', os.path.join(self.resource_path, 'beep.mp3')])
                    self.player.wait()
                elif full_text == "SCAN::":
                    self.stop_audio()
                    # [수정] -autoexit 제거, -loop 0 추가로 무한 반복
                    self.player = subprocess.Popen([
                        'ffplay', '-nodisp', '-loop', '0', '-hide_banner', 
                        os.path.join(self.resource_path, 'drum.mp3')
                    ])
                else:
                    self.stop_audio()
                    tts = gTTS(text=full_text, lang='ko')
                    temp_filename = "temp_voice.mp3"
                    tts.save(temp_filename)
                    # 재생 속도 1.5배 설정
                    self.player = subprocess.Popen(['ffplay', '-nodisp', '-autoexit', '-hide_banner', '-af', 'atempo=1.00', temp_filename])
                    self.player.wait()
                    if os.path.exists(temp_filename): os.remove(temp_filename)
            except Exception as e:
                self.get_logger().error(f"TTS Error: {e}")
            finally:
                self.msg_queue.task_done()

def main():
    rclpy.init()
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()