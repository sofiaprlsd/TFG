#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np

class SignalPlotterNode(Node):
    def __init__(self):
        super().__init__('signal_plotter_node')

        self.subscription = self.create_subscription(
            Float32,
            'PositionReference',
            self.listener_callback,
            10
        )

        self.time = 0.0
        self.window_size_x = 2
        self.window_size_y = 10
        self.sample_amount = 500

        self.time_data = np.zeros(self.sample_amount)
        self.signal_data = np.zeros(self.sample_amount)

        plt.ion()   # Update graph in real time
        self.fig, self.ax = plt.subplots(figsize=(12, 8))

        self.line, = self.ax.plot(self.time_data, self.signal_data, label='Signal')

        self.ax.set_xlim(0, self.window_size_x)
        self.ax.set_ylim(-self.window_size_y, self.window_size_y)
        self.ax.set_title(f'Signal')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Amplitude')
    
    def listener_callback(self, msg):
        self.time_data = np.roll(self.time_data, -1)
        self.signal_data = np.roll(self.signal_data, -1)

        self.time_data[-1] = self.time
        self.signal_data[-1] = msg.data

        self.time += 0.01

        if self.time_data[-1] > self.window_size_x:
            self.ax.set_xlim(self.time - self.window_size_x, self.time)

        self.line.set_xdata(self.time_data)
        self.line.set_ydata(self.signal_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    signal_plotter_node = SignalPlotterNode()

    try:
        rclpy.spin(signal_plotter_node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        signal_plotter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()