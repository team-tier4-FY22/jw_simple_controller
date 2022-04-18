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
# カスタムメッセージ
from jw_interface_msgs.msg import CommandStamped, Command, ModeCommand, JSADCommand, MotorRPMCommand
from std_msgs.msg import Header
class MyPublisher(Node):
    """
    送信側
    """

    # ノード名
    SELFNODE = "minoda_mypub"
    # トピック名
    SELFTOPIC = "/jw/command"

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
        self.pub = self.create_publisher(CommandStamped, self.SELFTOPIC, 10)
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
        # 送信するメッセージの作成
        msg = CommandStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command.mode.mode = 1
        msg.command.js_ad.front_back_ratio = -20.0
        msg.command.js_ad.left_right_ratio = 0.0
        # msg.command.motor_rpm.left_rpm = 100.0
        # msg.command.motor_rpm.right_rpm = 100.0
        # 送信
        self.pub.publish(msg)

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