import sys ## 터미널 창으로부터 두 정수를 입력 받기

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')

        ### ============= 클라이언트 설정 =============
        self.cli = self.create_client(
            AddTwoInts, ## 서비스 타입
            'add_two_ints') ## 서비스 명
        ### ===========================================

        while not self.cli.wait_for_service(timeout_sec=1.0): ## 일치하는 service client self.cli가 사용 가능한지 1초에 한번씩 확인
            self.get_logger().info('service not available, waiting again...')


    
    ### ============= request를 보내는 함수 =============
    def send_request(self, a, b): 
        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    ### =========================================== 
   

def main(args=None):
    rclpy.init(args=args) # 초기화
    minimal_client= MinimalClientAsync() # MinimalClientAsync를 node라는 이름으로 생성
    try:
        response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))  ## v1
        minimal_client.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (int(sys.argv[1]), int(sys.argv[2]), response.sum))        
    except KeyboardInterrupt:
        minimal_client.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:# 종료시 (ex `Ctrl + c`)
        minimal_client.destroy_node()  # 노드 소멸
        rclpy.shutdown() # rclpy.shutdown 함수로 노드 종료


if __name__ == '__main__':
    main()