#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
from pynput import keyboard
import random

class FlappyBirdNode(Node):
    def __init__(self):
        super().__init__('flappy_bird_node')

        self.signal_subscriber_ = self.create_subscription(
            Float32,
            'CleanSignal',
            self.listener_callback,
            10
        )

        self.params_subscriber_ = self.create_subscription(
            Int32,
            'GameParameters',
            self.level_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'PlayerPosition',
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
        self.max_offset = 3.0
        self.min_offset = 1.0
        self.level = 1
        self.max_level = 10

        self.game_completed = False

        self.time_data = np.zeros(self.sample_amount)
        self.signal_data = np.zeros(self.sample_amount)
        self.signal_upper = np.zeros(self.sample_amount)
        self.signal_lower = np.zeros(self.sample_amount)

        plt.ion()   # Update graph in real time
        self.fig, self.ax = plt.subplots(figsize=(12, 8))

        self.line, = self.ax.plot(self.time_data, self.signal_data, color='blue', label='Signal')
        self.line_upper, = self.ax.plot(self.time_data, self.signal_upper, linestyle='--', color='orange', label='Signal + offset')
        self.line_lower, = self.ax.plot(self.time_data, self.signal_lower, linestyle='--', color='orange', label='Signal - offset')
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
    
    def level_callback(self, msg):
        self.level = msg.data
        self.game_completed = False
        self.ax.set_title(f'Flappy Bird Level {self.level}')
        self.get_logger().info(f'Received new level: {self.level}')
    
    def listener_callback(self, msg):
        self.time_data = np.roll(self.time_data, -1)
        self.signal_data = np.roll(self.signal_data, -1)
        self.signal_upper = np.roll(self.signal_upper, -1)
        self.signal_lower = np.roll(self.signal_lower, -1)

        self.time_data[-1] = self.time
        self.signal_data[-1] = msg.data
        self.signal_upper[-1] = msg.data + self.offset_y
        self.signal_lower[-1] = msg.data - self.offset_y

        upper_limit = self.signal_upper[-1]
        lower_limit = self.signal_lower[-1]

        # Reduce the offset of the signals to make the game more difficult, 
        # only if the player is not colliding with any limit (signals)
        if not (lower_limit < self.player_y < upper_limit):
            self.get_logger().info("Collision!!!")
        else:
            if not self.game_completed:
                if self.level == self.max_level and self.offset_y <= self.min_offset:
                    self.game_completed = True
                    self.get_logger().info("GAME COMPLETED!")
                else:
                    reduction_factor = 0.005 * self.level # more level, bigger reduction
                    if self.time % 1.0 < 0.01 and self.offset_y > self.min_offset:
                        self.offset_y -= reduction_factor
                        self.get_logger().info(f"Reducing offset: {self.offset_y:.2f}")
                    if self.offset_y < (self.max_offset - self.level * 0.2) and self.level < self.max_level:
                        self.level += 1
                        self.ax.set_title(f'Flappy Bird Level {self.level}')
                        self.get_logger().info(f"Upgrade to level {self.level}")

        # When time is greater than window size, 
        # increment player_x to stay in same spot of the window
        self.time += 0.01
        if self.time >= self.window_size_x:
            self.player_x += 0.01

        if self.time_data[-1] > self.window_size_x:
            self.ax.set_xlim(self.time - self.window_size_x, self.time)

        # Plot sigal and player position
        self.line.set_xdata(self.time_data)
        self.line.set_ydata(self.signal_data)
        self.line_upper.set_xdata(self.time_data)
        self.line_upper.set_ydata(self.signal_upper)
        self.line_lower.set_xdata(self.time_data)
        self.line_lower.set_ydata(self.signal_lower)
        self.player.set_xdata(self.player_x)
        self.player.set_ydata(self.player_y)

        # Change color of the dot based on the distance from the limits
        dist = abs(self.player_y)
        max_safe_dist = self.offset_y
        norm_dist = min(dist / max_safe_dist, 1.0) # max 1.0
        if norm_dist < 0.4:
            color = (0, 1, 0) # green == safe
        elif norm_dist < 0.7:
            color = (1, 1, 0) # yellow == careful
        else:
            color = (1, 0, 0) # red == critic
        self.player.set_color(color)
        
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Publish player position on topic
        pos_msg = Float32MultiArray()
        pos_msg.data = [self.player_x, self.player_y, self.offset_y, self.time]
        self.publisher_.publish(pos_msg)

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