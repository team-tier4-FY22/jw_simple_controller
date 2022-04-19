#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS2 Node 送信側
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import rclpy
from rclpy.node import Node

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand
from std_msgs.msg import Header
class MyPublisher(Node):
    """
    送信側
    """

    # ノード名
    SELFNODE = "minoda_mypub"
    # トピック名
    SELFTOPIC_ACK = "/control/command/control_cmd"
    SELFTOPIC_GEAR = "/control/command/gear_cmd"

    def __init__(self):
        """
        コンストラクタ
        Parameters
        ----------
        """
        # ノードの初期化
        super().__init__(self.SELFNODE)
        # コンソールに表示
        self.get_logger().info("%s initializing..." % (self.SELFNODE))
        # publisherインスタンスを生成
        self.pub_ack = self.create_publisher(AckermannControlCommand, self.SELFTOPIC_ACK, 10)
        self.pub_gear = self.create_publisher(GearCommand, self.SELFTOPIC_GEAR, 10)
        # タイマーのインスタンスを生成（1秒ごとに発生）
        self.create_timer(0.01, self.callback)

        # コンソールに表示
        self.get_logger().info("%s do..." % self.SELFNODE)

        self.count = 0

        # # call callback only once!!
        # self.callback()


    def __del__(self):
        """
        デストラクタ
        """
        # コンソールに表示
        self.get_logger().info("%s done." % self.SELFNODE)

    def callback(self):
        """
        タイマーの実行部
        """
        self.get_logger().info("Publish [%s]" % (self.count))
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = 2
        self.pub_gear.publish(msg)

        msg = AckermannControlCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.lateral.stamp = self.get_clock().now().to_msg()
        msg.lateral.steering_tire_angle = 1.57
        msg.lateral.steering_tire_rotation_rate = 0.0
        msg.longitudinal.stamp = self.get_clock().now().to_msg()
        msg.longitudinal.speed = 0.3
        msg.longitudinal.acceleration = 0.0
        msg.longitudinal.jerk = 0.0

        self.pub_ack.publish(msg)


        self.count += 1

def main(args=None):
    """
    メイン関数
    Parameters
    ----------
    """
    # try:
    #     # rclpyの初期化
    #     rclpy.init(args=args)
    #     # インスタンスを生成
    #     node = MyPublisher()
    #     # プロセス終了までアイドリング
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # rclpyの初期化
    rclpy.init(args=args)
    # インスタンスを生成
    node = MyPublisher()
    # プロセス終了までアイドリング
    rclpy.spin(node)
    # finally:
    #     # 終了処理
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()