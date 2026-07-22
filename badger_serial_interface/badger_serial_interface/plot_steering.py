#!/usr/bin/env python3
"""
Live plot of /badger/badger_status (raw 12-bit ADC steering value).
Run while the steering_interface node is active:
    python3 plot_steering.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

WINDOW = 2000   # number of samples to display


class SteeringPlotter(Node):
    def __init__(self):
        super().__init__('steering_plotter')
        self._buf = deque(maxlen=WINDOW)
        self.create_subscription(
            Int32,
            '/sensor/speed',
            self._cb,
            100,
        )

    def _cb(self, msg: Int32):
        self._buf.append(msg.data)


def main():
    rclpy.init()
    node = SteeringPlotter()

    fig, ax = plt.subplots(figsize=(10, 4))
    (line,) = ax.plot([], [], lw=1)
    ax.set_xlim(0, WINDOW)
    ax.set_ylim(-50, 4150)          # 12-bit ADC: 0-4095 + margin
    ax.set_title('Steering ADC – /badger/badger_status')
    ax.set_xlabel('Sample')
    ax.set_ylabel('Raw ADC value (12-bit)')
    ax.grid(True)

    def update(_frame):
        data = list(node._buf)
        line.set_data(range(len(data)), data)
        return (line,)

    def spin_once(_frame):
        rclpy.spin_once(node, timeout_sec=0)

    # Spin ROS callbacks every animation frame
    ani = animation.FuncAnimation(
        fig, lambda f: [spin_once(f), update(f)][-1],
        interval=50,   # ms between frames → ~20 Hz refresh
        blit=True,
        cache_frame_data=False,
    )

    try:
        plt.tight_layout()
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
