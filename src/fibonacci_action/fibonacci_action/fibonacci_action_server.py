import rclpy # Python ROS2 프로그래밍을 위한 rclpy 
from rclpy.node import Node# rclpy 의 Node 클래스 
from rclpy.action import ActionServer, GoalResponse # ActionServer와 GoalResponse import

from custom_action_interface.action import Fibonacci # 사전에 정의한 인터페이스 custom_action_interface import

import time

class FibonacciActionServer(Node):# Node 클래스를 상속
    def __init__(self):
        super().__init__('fibonacci_action_server') # 부모 클래스(Node)의 생성자를 호출하고 이름을 fibonacci_action_server로 지정

        ##==================== Action Server 정의 =========================
        self.action_server = ActionServer(
            self, #  실행 노드
            Fibonacci, # 메시지 타입
            'fibonacci',# 액션 이름(client도 동일하게 받아야함)
            self.execute_callback,
            goal_callback=self.goal_callback)
        ##==========================================================================

        self.get_logger().info("=== Fibonacci Action Server Started ====")

    ##=============== Goal Request 이후의 콜백함수 ===============
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback() 
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
            )

            print(f"Feedback: {feedback_msg.partial_sequence}")
            goal_handle.publish_feedback(feedback_msg) # 피드백 publish
            time.sleep(1)

        goal_handle.succeed() # 액션 client에 현재 액션 상태(성공) 알림
        self.get_logger().warn("==== Succeed ====")

        result = Fibonacci.Result()# Result로 선언
        result.sequence = feedback_msg.partial_sequence # feedback에 있던 값을 넘겨줌

        return result # 결과값을 반환
    ##===========================================================

    ##=============== Goal Request시  콜백함수 ===============
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')

        return GoalResponse.ACCEPT
    ##========================================================


def main(args=None):
    rclpy.init(args=args) # 초기화
    node = FibonacciActionServer() # FibonacciActionServer를 node라는 이름으로 생성
    try:
        rclpy.spin(node) # rclpy에게 이 Node를 반복해서 실행 (=spin) 하라고 전달
    except KeyboardInterrupt: # `Ctrl + c`가 동작했을 때
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown() # rclpy.shutdown 함수로 노드 종료


if __name__ == '__main__':
    main()

