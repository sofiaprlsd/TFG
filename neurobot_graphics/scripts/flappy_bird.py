#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np
from pynput import keyboard

class FlappyBirdNode(Node):
    def __init__(self):
        super().__init__('flappy_bird_node')

        self.subscription = self.create_subscription(
            Float32,
            'PositionReference',
            self.listener_callback,
            10
        )

        self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()

        self.time = 0.0
        self.window_size_x = 2
        self.window_size_y = 10
        self.sample_amount = 500
        self.player_x = 0.25
        self.player_y = 0.0
        self.offset_y = 3.0
        self.level = 0

        self.time_data = np.zeros(self.sample_amount)
        self.signal_upper = np.zeros(self.sample_amount)
        self.signal_lower = np.zeros(self.sample_amount)

        plt.ion()   # Update graph in real time
        self.fig, self.ax = plt.subplots(figsize=(12, 8))

        self.line_upper, = self.ax.plot(self.time_data, self.signal_upper, label='Signal + offset')
        self.line_lower, = self.ax.plot(self.time_data, self.signal_lower, label='Signal - offset')
        self.player, = self.ax.plot(self.player_x, self.player_y, 'ro', label='Player')

        self.ax.set_xlim(0, self.window_size_x)
        self.ax.set_ylim(-self.window_size_y, self.window_size_y)
        self.ax.set_title(f'Flappy Bird Level {self.level}')
    
    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.player_y += 0.2
            elif key == keyboard.Key.down:
                self.player_y -= 0.2
        except:
            pass
    
    def listener_callback(self, msg):
        self.time_data = np.roll(self.time_data, -1)
        self.signal_upper = np.roll(self.signal_upper, -1)
        self.signal_lower = np.roll(self.signal_lower, -1)

        self.time_data[-1] = self.time
        self.signal_upper[-1] = msg.data + self.offset_y
        self.signal_lower[-1] = msg.data - self.offset_y

        upper_limit = self.signal_upper[-1]
        lower_limit = self.signal_lower[-1]

        if not (lower_limit < self.player_y < upper_limit):
            self.get_logger().info("Collision!!!")
        else:
            if self.time % 1.0 < 0.01 and self.offset_y > 1.0:
                self.offset_y -= 0.05
                self.get_logger().info(f"Reducing offset: {self.offset_y:.2f}")
            if self.time % 5.0 < 0.01 and self.offset_y > 1.0:
                self.level += 1
                self.ax.set_title(f'Flappy Bird Level {self.level}')
                self.get_logger().info(f"Upgrade to level {self.level}")

        self.time += 0.01
        if self.time >= self.window_size_x:
            self.player_x += 0.01

        if self.time_data[-1] > self.window_size_x:
            self.ax.set_xlim(self.time - self.window_size_x, self.time)

        self.line_upper.set_xdata(self.time_data)
        self.line_upper.set_ydata(self.signal_upper)
        self.line_lower.set_xdata(self.time_data)
        self.line_lower.set_ydata(self.signal_lower)
        self.player.set_xdata(self.player_x)
        self.player.set_ydata(self.player_y)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    flappy_bird_node = FlappyBirdNode()

    try:
        rclpy.spin(flappy_bird_node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        flappy_bird_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()