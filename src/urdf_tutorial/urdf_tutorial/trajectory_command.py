#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
import sys

def main(args=None):
    rclpy.init(args=args)
    roscpp_initialize(sys.argv)

    # 로봇과 move group 초기화
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("robot_arm")  # move_group에 정의된 arm 그룹명

    # 현재 joint 값 확인
    current_joints = group.get_current_joint_values()
    print("현재 조인트 상태:", current_joints)

    # 목표 joint 설정 (예: 첫 번째 조인트 -0.5 rad 이동)
    target_joints = current_joints.copy()
    target_joints[0] -= 0.5

    # 이동 실행
    group.go(target_joints, wait=True)
    group.stop()

    # 종료
    roscpp_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
