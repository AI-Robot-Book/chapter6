import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
import time
import threading


# CRANE+用のアクションへリクエストを送るノード
class Commander(Node):

    def __init__(self, timer=False):
        super().__init__('commander')
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        self.publisher_joint = self.create_publisher(
            JointTrajectory,
            'crane_plus_arm_controller/joint_trajectory', 10)
        self.publisher_gripper = self.create_publisher(
            JointTrajectory,
            'crane_plus_gripper_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        self.lock = threading.Lock()
        self.joint = [0]*4
        self.gripper = 0
        if timer:
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
        self.action_client_joint = ActionClient(
            self, FollowJointTrajectory,
            'crane_plus_arm_controller/follow_joint_trajectory')

    def publish_joint(self, q, time):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_joint.publish(msg)

    def publish_gripper(self, gripper, time):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['crane_plus_joint_hand']
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [float(gripper)]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_gripper.publish(msg)

    def joint_state_callback(self, msg):
        d = {}
        for i, name in enumerate(msg.name):
            d[name] = msg.position[i]
        self.lock.acquire()
        self.joint = [d[x] for x in self.joint_names]
        self.gripper = d['crane_plus_joint_hand']
        self.lock.release()

    def get_joint_gripper(self):
        self.lock.acquire()
        j = self.joint.copy()
        g = self.gripper
        self.lock.release()
        return j, g

    def timer_callback(self):
        j, g = self.get_joint_gripper()
        print(f'[{j[0]:.2f}, {j[1]:.2f}, {j[2]:.2f}, {j[3]:.2f}] {g:.2f}')

    def send_goal_joint(self,  q, time):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [JointTrajectoryPoint()]
        goal_msg.trajectory.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        goal_msg.trajectory.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.action_client_joint.wait_for_server()
        return self.action_client_joint.send_goal(goal_msg)


def main(args=None):
    # ROSクライアントの初期化
    rclpy.init(args=args)

    # ノードクラスのインスタンス
    commander = Commander()

    # 別のスレッドでrclpy.spin()を実行する
    thread = threading.Thread(target=rclpy.spin, args=(commander,))
    thread.start()

    # 最初の指令をパブリッシュする前に少し待つ
    time.sleep(1.0)

    # 文字列とポーズの組を保持する辞書
    goals = {}
    goals['zeros'] = [0, 0, 0, 0]
    goals['ones'] = [1, 1, 1, 1]
    goals['home'] = [0.0, -1.16, -2.01, -0.73]
    goals['carry'] = [-0.00, -1.37, -2.52, 1.17]

    # 初期ポーズへゆっくり移動させる
    joint = [0.0, 0.0, 0.0, 0.0]
    gripper = 0
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    print('同期的なアクションの利用')

    # Ctrl+cでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            # 目標関節値とともに送る目標時間
            dt = 3.0

            for key, item in goals.items():
                print(f'{key:8} {item}')
            name = input('目標の名前を入力: ')
            if name == '':
                break
            if name not in goals:
                print(f'{name}は登録されていません')
                continue

            print('目標を送って結果待ち…')
            r = commander.send_goal_joint(goals[name], dt)
            print(f'r.result.error_code: {r.result.error_code}')
            j, g = commander.get_joint_gripper()
            print(f'[{j[0]:.2f}, {j[1]:.2f}, {j[2]:.2f}, {j[3]:.2f}] {g:.2f}')
            print('')
    except KeyboardInterrupt:
        pass

    # 終了ポーズへゆっくり移動させる
    joint = [0.0, 0.0, 0.0, 0.0]
    gripper = 0
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    rclpy.shutdown()
    print('終了')
