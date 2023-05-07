from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')

        ### ============= 서버 설정 =============
        self.srv = self.create_service(
            AddTwoInts, ## srv 타입: 해당 클래스의 인터페이스로 서비스 요청에 해당되는 request
            'add_two_ints', ## 서비스 서버 명
            self.add_two_ints_callback) ## 콜백 함수
        ### ===========================================

    def add_two_ints_callback(self, request, response): ## Client Node로부터 클래스로 생성된 인터페이스로 서비스 요청에 해당되는 request 부분과 응답에 해당되는 response으로 구분
        response.sum = request.a + request.b ## 위의 AddTwoInts 인터페이스 정보 확인
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b)) # cmd 창 출력

        return response ## 응답값 반환


def main(args=None):
    rclpy.init(args=args) # 초기화
    node = MinimalService() # MinimalService를 node라는 이름으로 생성
    try:
        rclpy.spin(node) # 생성한 노드를 spin하여 지정된 콜백 함수 실행 
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:# 종료시 (ex `Ctrl + c`)
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown() # rclpy.shutdown 함수로 노드 종료

if __name__ == '__main__':
    main()