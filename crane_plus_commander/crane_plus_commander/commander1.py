import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading
from crane_plus_commander.kbhit import KBHit
from crane_plus_commander.kinematics import gripper_in_range, joint_in_range


# CRANE+用のトピックへ指令をパブリッシュするノード
class Commander(Node):

    def __init__(self):
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
        # self.get_logger().info('Publishing: "%s"' % msg)

    def publish_gripper(self, gripper, time):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['crane_plus_joint_hand']
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [float(gripper)]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_gripper.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    # ROSクライアントの初期化
    rclpy.init(args=args)

    # ノードクラスのインスタンス
    commander = Commander()

    # 別のスレッドでrclpy.spin()を実行する
    thread = threading.Thread(
        target=rclpy.spin, args=(commander, ), daemon=True)
    thread.start()

    # 最初の指令をパブリッシュする前に少し待つ
    time.sleep(1.0)

    # 指令値
    joint = [0.0, 0.0, 0.0, 0.0]
    gripper = 0
    dt = 5

    # 最初にゆっくり初期状態へ移動する
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    # キー読み取りクラスのインスタンス
    kb = KBHit()

    print('1, 2, 3, 4, 5, 6, 7, 8, 9, 0キーを押して関節を動かす')
    print('スペースキーを押して起立状態にする')
    print('Escキーを押して終了')

    # Ctrl+cでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            # 変更前の値を保持
            joint_prev = joint.copy()
            gripper_prev = gripper

            # 目標関節値とともに送る目標時間
            dt = 0.2

            # キーが押されているか？
            if kb.kbhit():
                c = kb.getch()
                # 押されたキーによって場合分けして処理
                if c == '1':
                    joint[0] -= 0.1
                elif c == '2':
                    joint[0] += 0.1
                elif c == '3':
                    joint[1] -= 0.1
                elif c == '4':
                    joint[1] += 0.1
                elif c == '5':
                    joint[2] -= 0.1
                elif c == '6':
                    joint[2] += 0.1
                elif c == '7':
                    joint[3] -= 0.1
                elif c == '8':
                    joint[3] += 0.1
                elif c == '9':
                    gripper -= 0.1
                elif c == '0':
                    gripper += 0.1
                elif c == ' ':  # スペースキー
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0
                    dt = 1.0
                elif ord(c) == 27:  # Escキー
                    break

                # 指令値を範囲内に収める
                if not joint_in_range(joint) == [True]*4:
                    print('関節指令値が範囲外')
                    joint = joint_prev.copy()
                if not gripper_in_range(gripper):
                    print('グリッパ指令値が範囲外')
                    gripper = gripper_prev

                # 変化があればパブリッシュ
                publish = False
                if joint != joint_prev:
                    print((f'joint: [{joint[0]:.2f}, {joint[1]:.2f}, '
                           f'{joint[2]:.2f}, {joint[3]:.2f}]'))
                    commander.publish_joint(joint, dt)
                    publish = True
                if gripper != gripper_prev:
                    print(f'gripper: {gripper:.2f}')
                    commander.publish_gripper(gripper, dt)
                    publish = True
                # パブリッシュした場合は設定時間の分停止
                if publish:
                    time.sleep(dt)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass

    # 後始末
    joint = [0.0, 0.0, 0.0, 0.0]
    gripper = 0
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    rclpy.shutdown()
    thread.join()
    print('終了')


if __name__ == '__main__':
    main()
