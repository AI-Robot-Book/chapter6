import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.srv import StringCommand
from crane_plus_commander.kinematics import (
    from_gripper_ratio, gripper_in_range)


# CRANE+用のアクションへリクエストを送り，他からサービスを受け付けるノード
class Commander(Node):

    def __init__(self, timer=False):
        super().__init__('commander')
        self.callback_group = ReentrantCallbackGroup()
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        self.gripper_names = [
            'crane_plus_joint_hand']
        self.action_client_joint = ActionClient(
            self, FollowJointTrajectory,
            'crane_plus_arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
        self.action_client_gripper = ActionClient(
            self, FollowJointTrajectory,
            'crane_plus_gripper_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
        self.action_done_event = Event()
        # 文字列とポーズの組を保持する辞書
        self.poses = {}
        self.poses['zeros'] = [0, 0, 0, 0]
        self.poses['ones'] = [1, 1, 1, 1]
        self.poses['home'] = [0.0, -1.16, -2.01, -0.73]
        self.poses['carry'] = [-0.00, -1.37, -2.52, 1.17]
        self.service = self.create_service(
            StringCommand, 'command', self.command_callback,
            callback_group=self.callback_group)

    def command_callback(self, request, response):
        self.get_logger().info(f'command: {request.command}')
        words = request.command.split()
        if words[0] == 'set_pose':
            self.set_pose(words, response)
        elif words[0] == 'set_gripper':
            self.set_gripper(words, response)
        else:
            response.answer = f'NG {words[0]} not supported'
        self.get_logger().info(f'answer: {response.answer}')
        return response

    def set_pose(self, words, response):
        if len(words) < 2:
            response.answer = f'NG {words[0]} argument required'
            return
        if not words[1] in self.poses:
            response.answer = f'NG {words[1]} not found'
            return
        r = self.send_goal_joint(self.poses[words[1]], 3.0)
        if self.check_action_result(r, response):
            return
        response.answer = 'OK'

    def set_gripper(self, words, response):
        if len(words) < 2:
            response.answer = f'NG {words[0]} argument required'
            return
        try:
            gripper_ratio = float(words[1])
        except ValueError:
            response.answer = f'NG {words[1]} unsuitable'
            return
        gripper = from_gripper_ratio(gripper_ratio)
        if not gripper_in_range(gripper):
            response.answer = 'NG out of range'
            return
        dt = 1.0
        r = self.send_goal_gripper(gripper, dt)
        if self.check_action_result(r, response):
            return
        response.answer = 'OK'

    def check_action_result(self, r, response, message=''):
        if message != '':
            message += ' '
        if r is None:
            response.answer = f'NG {message}timeout'
            return True
        if r.result.error_code != 0:
            response.answer = f'NG {message}error_code: {r.result.error_code}'
            return True
        return False

    def send_goal_joint(self, q, time):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [JointTrajectoryPoint()]
        goal_msg.trajectory.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        goal_msg.trajectory.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.action_done_event.clear()
        send_goal_future = self.action_client_joint.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.action_done_event.wait(time*2)
        return self.action_result

    def send_goal_gripper(self, gripper, time):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory.joint_names = self.gripper_names
        goal_msg.trajectory.points = [JointTrajectoryPoint()]
        goal_msg.trajectory.points[0].positions = [float(gripper)]
        goal_msg.trajectory.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.action_done_event.clear()
        send_goal_future = self.action_client_gripper.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.action_done_event.wait(time*2)
        return self.action_result

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result = future.result()
        self.action_done_event.set()


def main(args=None):
    print('開始')

    # ROSクライアントの初期化
    rclpy.init(args=args)

    # ノードクラスのインスタンス
    commander = Commander()

    # 初期ポーズへゆっくり移動させる
    commander.send_goal_joint(commander.poses['home'], 5)
    commander.send_goal_gripper(from_gripper_ratio(1), 1)
    print('サービスサーバ待機')

    # Ctrl+cでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(commander, executor)
    except KeyboardInterrupt:
        pass

    print('サービスサーバ停止')
    # 終了ポーズへゆっくり移動させる
    commander.send_goal_joint(commander.poses['zeros'], 5)
    commander.send_goal_gripper(from_gripper_ratio(0), 1)

    rclpy.shutdown()
    print('終了')
