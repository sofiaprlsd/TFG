#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class FlappyBirdViewerNode(Node):
    def __init__(self):
        super().__init__('flappy_bird_viewer_node')

        self.signal_subscriber_ = self.create_subscription(
            Float32,
            'PositionReference',
            self.signal_callback,
            10
        )

        self.player_subscriber_ = self.create_subscription(
            Float32MultiArray,
            'PlayerPosition',
            self.player_callback,
            10
        )
        
        self.time = 0.0
        self.window_size_x = 2
        self.window_size_y = 10
        self.sample_amount = 500
        self.player_x = 0.25
        self.player_y = 0.0
        self.offset_y = 3.0

        self.player_active = False

        self.time_data = np.zeros(self.sample_amount)
        self.signal_upper = np.zeros(self.sample_amount)
        self.signal_lower = np.zeros(self.sample_amount)

        plt.ion()   # Update graph in real time
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

        self.ax.set_facecolor('black')
        self.ax.set_title('Flappy Bird Viewer - Waiting player...')

        self.line_upper, = self.ax.plot([], [], label='Signal + offset')
        self.line_lower, = self.ax.plot([], [], label='Signal - offset')
        self.player, = self.ax.plot([], [], 'ro', label='Player')

        self.ax.set_xlim(0, self.window_size_x)
        self.ax.set_ylim(-self.window_size_y, self.window_size_y)

        # Show window
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def signal_callback(self, msg):
        if not self.player_active:
            return
        
        self.ax.set_facecolor('white')
        self.ax.set_title(f'Flappy Bird Viewer | Offset: {self.offset_y:.2f}')
        
        self.time_data = np.roll(self.time_data, -1)
        self.signal_upper = np.roll(self.signal_upper, -1)
        self.signal_lower = np.roll(self.signal_lower, -1)

        self.time_data[-1] = self.time
        self.signal_upper[-1] = msg.data + self.offset_y
        self.signal_lower[-1] = msg.data - self.offset_y

        upper_limit = self.signal_upper[-1]
        lower_limit = self.signal_lower[-1]

        # Check if the player is colliding with any limit (signals)
        if not (lower_limit < self.player_y < upper_limit):
            self.get_logger().info("Collision!!!")
        
        if self.time_data[-1] > self.window_size_x:
            self.ax.set_xlim(self.time - self.window_size_x, self.time)

        # Plot sigal and player position
        self.line_upper.set_xdata(self.time_data)
        self.line_upper.set_ydata(self.signal_upper)
        self.line_lower.set_xdata(self.time_data)
        self.line_lower.set_ydata(self.signal_lower)
        self.player.set_xdata(self.player_x)
        self.player.set_ydata(self.player_y)

        # Adjust graph to the updated data
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def player_callback(self, msg):
        if len(msg.data) >= 3:
            self.player_x = msg.data[0]
            self.player_y = msg.data[1]
            self.offset_y = msg.data[2]
            self.time = msg.data[3]
            self.player_active = True

def main(args=None):
    rclpy.init(args=args)
    flappy_bird_viewer_node = FlappyBirdViewerNode()

    try:
        rclpy.spin(flappy_bird_viewer_node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        flappy_bird_viewer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()