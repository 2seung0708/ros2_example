import rclpy # Python ROS2 프로그래밍을 위한 rclpy 
from rclpy.node import Node# rclpy 의 Node 클래스 
from rclpy.action import ActionClient, GoalResponse # ActionServer와 GoalResponse import

from custom_action_interface.action import Fibonacci # 사전에 정의한 인터페이스 custom_action_interface import

class FibonacciActionClient(Node):# Node 클래스를 상속
    def __init__(self):
        super().__init__('fibonacci_action_client') # 부모 클래스(Node)의 생성자를 호출하고 이름을 fibonacci_action_client로 지정

        ##==================== Action Server 정의 =========================
        self.action_client = ActionClient(
            self, #  실행 노드
            Fibonacci, # 메시지 타입
            'fibonacci')# 액션 이름(action server에서 정한 이름과 동일해야함!)
        ##==========================================================================

        self.get_logger().info("=== Fibonacci Action Client Started ====")

    ##=============== send Gaol ===============
    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # 10초간 server를 기다리다가 응답이 없으면 에러를 출력
        if self.action_client.wait_for_server(10) is False:
            self.get_logger().error("Server Not exists")
        
        # goal request가 제대로 보내졌는지 알기 위해 future가 사용됩니다.
        # 더불어, feedback_callback을 묶어 feedback 발생 시 해당 함수로 이동합니다.
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        
        # server가 존재한다면, Goal Request의 성공 유무,
        # 최종 Result에 대한 callback도 필요
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    ##===========================================================

    ##=============== feedback을 받아오는 함수 ===============
    def feedback_callback(self, feedback_msg):#send_goal에 사용됨
        feedback = feedback_msg.feedback
        print(f"Received feedback: {feedback.partial_sequence}") # 출력
    ##========================================================


    ##=============== Goal Request에 대한 응답으로 실행되는 callback ===============
    def goal_response_callback(self, future):
        goal_handle = future.result()

        # Goal type에 따라 성공 유무를 판단합니다.
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
				
        # 최종 Result 데이터를 다룰 callback을 연동합니다.
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    ##========================================================

    ##===============  Result callback 함수 ===============
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().warn(f"Action Done !! Result: {result.sequence}")
        rclpy.shutdown()
    ##========================================================

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_client = FibonacciActionClient()
		
    # Client Node 생성 이후 직접 send_goal을 해줍니다. (Service와 유사)
    # Goal Request에 대한 future를 반환받음
    future = fibonacci_action_client.send_goal(5)

    rclpy.spin(fibonacci_action_client)

if __name__ == '__main__':
    main()

