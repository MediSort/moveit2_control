import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class ArmActionClient(Node):

    def __init__(self):
        super().__init__('arm_action_client')
        # 액션 클라이언트 초기화
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/robot_arm_controller/follow_joint_trajectory'
        )

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        # ✅ 로봇 조인트 이름을 실제 URDF / MoveIt 설정에 맞게 넣어주세요
        goal_msg.trajectory.joint_names = [
            'joint1_base_joint1', 'joint2_base_joint2', 'joint3_base_joint3_y', 'joint3_frame_joint3_z', 'joint4_base_joint4_y', 'joint4_frame_joint4_z'
        ]

        # 목표 포인트 정의
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.0, 1.0, 0.0, 0.5, 0.0]  # 라디안 값
        point.time_from_start.sec = 3   # 3초 안에 도달

        goal_msg.trajectory.points.append(point)

        # 서버 준비 대기
        self._action_client.wait_for_server()

        # Goal 전송
        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ Goal rejected')
            return

        self.get_logger().info('✅ Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🎯 Result: {result}')


def main(args=None):
    rclpy.init(args=args)
    action_client = ArmActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
