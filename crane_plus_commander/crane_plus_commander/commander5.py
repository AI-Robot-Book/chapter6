import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading
from math import pi
from crane_plus_commander.kbhit import KBHit
from crane_plus_commander.kinematics import (
    forward_kinematics, from_gripper_ratio, gripper_in_range,
    inverse_kinematics, joint_in_range, to_gripper_ratio)


# CRANE+用のトピックへ指令をパブリッシュし，tfを利用するノード
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
        self._tf_publisher = StaticTransformBroadcaster(self)
        self.send_static_transform()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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

    def send_static_transform(self):
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'crane_plus_link4'
        st.child_frame_id = 'crane_plus_endtip'
        st.transform.translation.x = 0.0
        st.transform.translation.y = 0.0
        st.transform.translation.z = 0.090
        qu = quaternion_from_euler(0.0, -pi/2, 0.0)
        st.transform.rotation.x = qu[0]
        st.transform.rotation.y = qu[1]
        st.transform.rotation.z = qu[2]
        st.transform.rotation.w = qu[3]
        self._tf_publisher.sendTransform(st)

    def get_endtip_position(self):
        try:
            when = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(
                'crane_plus_base',
                'crane_plus_endtip',
                when,
                timeout=Duration(seconds=1.0))
        except LookupException as e:
            self.get_logger().info(e)
            return None
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        rx = trans.transform.rotation.x
        ry = trans.transform.rotation.y
        rz = trans.transform.rotation.z
        rw = trans.transform.rotation.w
        roll, pitch, yaw = euler_from_quaternion([rx, ry, rz, rw])
        return tx, ty, tz, roll, pitch, yaw


def main():
    # ROSクライアントの初期化
    rclpy.init()

    # ノードクラスのインスタンス
    commander = Commander()

    # 別のスレッドでrclpy.spin()を実行する
    thread = threading.Thread(target=rclpy.spin, args=(commander,))
    thread.start()

    # 最初の指令をパブリッシュする前に少し待つ
    time.sleep(2.0)

    # 初期ポーズへゆっくり移動させる
    joint = [0.0, -1.16, -2.01, -0.73]
    gripper = 0
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    # 逆運動学の解の種類
    elbow_up = True

    # キー読み取りクラスのインスタンス
    kb = KBHit()

    print('1, 2, 3, 4, 5, 6, 7, 8, 9, 0キーを押して関節を動かす')
    print('a, z, s, x, d, c, f, v, g, bキーを押して手先を動かす')
    print('eキーを押して逆運動学の解を切り替える')
    print('スペースキーを押して起立状態にする')
    print('Escキーを押して終了')

    # Ctrl+cでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            # 順運動学
            [x, y, z, pitch] = forward_kinematics(joint)
            ratio = to_gripper_ratio(gripper)
            # 変更前の値を保持
            joint_prev = joint.copy()
            gripper_prev = gripper
            elbow_up_prev = elbow_up

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
                elif c == 'a':
                    x += 0.01
                elif c == 'z':
                    x -= 0.01
                elif c == 's':
                    y += 0.01
                elif c == 'x':
                    y -= 0.01
                elif c == 'd':
                    z += 0.01
                elif c == 'c':
                    z -= 0.01
                elif c == 'f':
                    pitch += 0.1
                elif c == 'v':
                    pitch -= 0.1
                elif c == 'g':
                    ratio += 0.1
                elif c == 'b':
                    ratio -= 0.1
                elif c == 'e':
                    elbow_up = not elbow_up
                    print(f'elbow_up: {elbow_up}')
                    dt = 3.0
                elif c == ' ':  # スペースキー
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0
                    dt = 1.0
                elif ord(c) == 27:  # Escキー
                    break

                # 逆運動学
                if c in 'azsxdcfve':
                    joint = inverse_kinematics([x, y, z, pitch], elbow_up)
                    if joint is None:
                        print('逆運動学の解なし')
                        joint = joint_prev.copy()
                elif c in 'gb':
                    gripper = from_gripper_ratio(ratio)

                # 指令値を範囲内に収める
                if not all(joint_in_range(joint)):
                    print('関節指令値が範囲外')
                    joint = joint_prev.copy()
                    elbow_up = elbow_up_prev
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
                    position = commander.get_endtip_position()
                    if position is not None:
                        x, y, z, roll, pitch, yaw = position
                        print((f'x: {x:.3f}, y: {y:.3f}, z: {z:.3f}, '
                               f'roll: {roll:.3f}, pitch: {pitch:.3f}, '
                               f'yaw: {yaw:.3f}'))
            time.sleep(0.01)
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
