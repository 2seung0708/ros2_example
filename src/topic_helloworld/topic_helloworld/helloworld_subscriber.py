import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile # QoS 설정을 위해 
from std_msgs.msg import String # String 메시지 인터페이스를 사용하기 위해 import

class HelloworldSubscriber(Node): # Node 클래스를 상속
    def __init__(self):
        super().__init__('Helloworld_subscriber') # 부모 클래스(Node)의 생성자를 호출하고 이름을 helloworld_publisher로 지정
        qos_profile = QoSProfile(depth=10) # 통신상태가 원활하지 못할 경우 퍼블리시 할 데이터를 버퍼에 10개까지 저장하라는 설정
        self.helloworld_subscriber = self.create_subscription( # create_subscription함수를 이용해 helloworld_subscriber 설정
            String, # 토픽 메시지 타입
            'helloworld', # 토픽 이름
            self.subscribe_topic_message, # 수신 받은 메세지를 처리할 콜백함수 **
            qos_profile) # QoS: qos_profile

    def subscribe_topic_message(self, msg): # 앞서 지정한 콜백 함수 정의. 
        self.get_logger().info('Received message: {0}'.format(msg.data))  # 터미널 창에 출력하며 로그 기록됨 (python의 print도 가능)

def main(args=None):
    rclpy.init(args=args) # 초기화
    node = HelloworldSubscriber() # HelloworldSubscriber를 node라는 이름으로 생성
    try:
        rclpy.spin(node) # rclpy에게 이 Node를 반복해서 실행 (=spin) 하라고 전달
    except KeyboardInterrupt: # `Ctrl + c`가 동작했을 때
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown() # rclpy.shutdown 함수로 노드 종료


if __name__ == '__main__':
    main()
