import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile # QoS 설정을 위해 
from std_msgs.msg import String # String 메시지 인터페이스를 사용하기 위해 import

class HelloworldPublisher(Node): # Node 클래스를 상속
    def __init__(self):
        super().__init__('helloworld_publisher') # 부모 클래스(Node)의 생성자를 호출하고 이름을 helloworld_publisher로 지정
        qos_profile = QoSProfile(depth=10) # 통신상태가 원활하지 못할 경우 퍼블리시 할 데이터를 버퍼에 10개까지 저장하라는 설정
        self.helloworld_publisher = self.create_publisher(# create_publisher함수를 이용해 helloworld_publisher 설정
            String, # 토픽 메시지 타입: String,
            'helloworld', #  토픽 이름: helloworld 
            qos_profile)#, QoS설정
        self.timer = self.create_timer(1, self.publish_helloworld_msg) # 콜백 함수를 실행. 1초마다 publish_helloworld_msg 함수 실행
        self.count = 0

    def publish_helloworld_msg(self): # 앞서 지정한 콜백 합수
        msg = String() # 메시지 타입 String으로 지정
        msg.data = 'Hello World: {0}'.format(self.count) # 표준 문자열 메세지 구조에 따라 해당 문자열을 msg.data에 담아줌
        self.helloworld_publisher.publish(msg) # __init__ 에서 정의한 helloworld_publisher 퍼블리시!
        self.get_logger().info('Published message: {0}'.format(msg.data)) # 터미널 창에 출력하며 로그 기록됨 (python의 print도 가능)
        self.count += 1


def main(args=None):
    rclpy.init(args=args) # 초기화
    node = HelloworldPublisher() # HelloworldPublisher를 node라는 이름으로 생성
    try:
        rclpy.spin(node) # rclpy에게 이 Node를 반복해서 실행 (=spin) 하라고 전달
    except KeyboardInterrupt: # `Ctrl + c`가 동작했을 때
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown() # rclpy.shutdown 함수로 노드 종료


if __name__ == '__main__':
    main()
