#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
from pynput import keyboard
import pygame
import os

pygame.mixer.init()
level_up_sound = pygame.mixer.Sound('../sounds/level_up.wav')

class FlappyBirdNode(Node):
    def __init__(self):
        super().__init__('flappy_bird_node')

        self.signal_subscriber_ = self.create_subscription(
            Float32,
            'CleanSignal',
            self.listener_callback,
            10
        )

        self.level_subscriber_ = self.create_subscription(
            Int32,
            'GameParameters',
            self.level_callback,
            10
        )

        self.offset_subscriber_ = self.create_subscription(
            Float32MultiArray,
            'SliderParameters',
            self.offset_callback,
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
        self.score = 0
        self.start_time = None
        self.inside_limits = True
        self.obj_interval = 3.0
        self.last_obj_time = 0.0
        self.max_active_obj = 2
        self.obj_radius_x = 0.08
        self.obj_radius_y = 0.3
        self.obj_counter = 0
        self.objects = []
        self.plot_objects = []
        self.collected = set()
        self.inverted_gravity = False
        self.mission_completed = False
        self.game_completed = False

        self.level_colors = [
            {'bg': '#e0f7fa', 'line': '#00796b'},    # light cian / teal
            {'bg': '#fff8e1', 'line': '#f57f17'},    # light yellow / amber
            {'bg': '#e8f5e9', 'line': '#388e3c'},    # light green / green
            {'bg': '#fce4ec', 'line': '#c2185b'},    # light pink / pink
            {'bg': '#ede7f6', 'line': '#5e35b1'},    # light purple / purple
            {'bg': '#f3e5f5', 'line': '#7b1fa2'},    # lavender / violet
            {'bg': '#fbe9e7', 'line': '#e64a19'},    # peech / deep orange
            {'bg': '#f1f8e9', 'line': '#689f38'},    # pistachio / olive
            {'bg': '#e0f2f1', 'line': '#00695c'},    # pale turquoise / strong teal
            {'bg': '#e3f2fd', 'line': '#1976d2'},    # light blue / royal
        ]

        self.time_data = np.zeros(self.sample_amount)
        self.signal_data = np.zeros(self.sample_amount)
        self.signal_upper = np.zeros(self.sample_amount)
        self.signal_lower = np.zeros(self.sample_amount)

        plt.ion()   # Update graph in real time
        self.fig, self.ax = plt.subplots(figsize=(12, 8))

        self.score_text = self.ax.text(0.95, 0.9, f"Score{self.score}", transform=self.ax.transAxes, fontsize=12, color='black', ha='right')
        self.mission_text = self.ax.text(0.05, 0.9, "", transform=self.ax.transAxes, fontsize=12, color='black', ha='left')

        self.line, = self.ax.plot(self.time_data, self.signal_data, color='lightblue', label='Signal')
        self.line_upper, = self.ax.plot(self.time_data, self.signal_upper, linestyle='--', color='white', label='Signal + offset')
        self.line_lower, = self.ax.plot(self.time_data, self.signal_lower, linestyle='--', color='white', label='Signal - offset')
        self.player, = self.ax.plot(self.player_x, self.player_y, 'ro', label='Player')
        
        self.ax.set_xlim(0, self.window_size_x)
        self.ax.set_ylim(-self.window_size_y, self.window_size_y)
        # Do not show axis numbers
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.update_level_effects()
    
    def update_level_effects(self):
        colors = self.level_colors[(self.level - 1) % len(self.level_colors)]
        self.ax.set_facecolor(colors['bg'])
        self.line.set_color(colors['line'])
        self.ax.set_title(f'Flappy Bird Level {self.level}')
        level_up_sound.play()
        self.fig.canvas.draw()
        self.get_logger().info(f"Upgrade to level {self.level}")
    
    def increment_score(self, points):
        self.score += points
        self.score_text.set_text(f"Score: {self.score}")
    
    def decrement_score(self, points):
        self.score -= points
        if self.score < 0:
            self.score = 0
        self.score_text.set_text(f"Score: {self.score}")
    
    def clear_objects(self):
        for s in self.plot_objects:
            s.remove()
        self.objects = []
        self.plot_objects = []
        self.collected = set()
    
    def generate_star(self):
        x = self.time + np.random.uniform(0.5, 4.0)
        y = 0.0
        if self.time_data[-1] > self.time_data[0]:
            y = np.interp(x, self.time_data, self.signal_data)
        self.objects.append([x, y])
        o = self.ax.plot(x, y, marker="*", color="gold", markersize=25)[0]
        self.plot_objects.append(o)
    
    def generate_asteroid(self):
        x = self.time + np.random.uniform(0.5, 4.0)
        y = np.random.uniform(1.0, self.offset_y)
        if np.random.rand() > 0.5:
            y = np.random.uniform(-self.offset_y, -1.0)
        self.objects.append([x, y])
        o = self.ax.plot(x, y, marker="o", color="brown", markersize=25)[0]
        self.plot_objects.append(o)
    
    def on_press(self, key):
        try:
            direction = -1 if self.inverted_gravity else 1
            if key == keyboard.Key.up:
                self.player_y += 0.1 * direction
            elif key == keyboard.Key.down:
                self.player_y -= 0.1 * direction
        except:
            pass
    
    def level_callback(self, msg):
        self.level = msg.data[0]
        self.get_logger().info(f'Received L{self.level}')

        self.mission_completed = False
        self.game_completed = False
        self.inside_limits = True
        self.start_time = None
        self.update_level_effects()

        if self.level in [2, 6]:
            self.clear_objects()
            self.obj_counter = 0
            self.generate_star()
            self.last_obj_time = self.time
    
    def offset_callback(self, msg):
        self.offset_y = msg.data[2]
        self.get_logger().info(f'Current O{self.offset_y}')
    
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
            self.score_text.set_text(f"Score: {self.score}")
            if self.level in [1, 4, 5, 7, 9]:
                mission_time = {1: 20.0, 4: 40.0, 5: 10.0, 7: 60.0, 9: 15.0}[self.level]
                if self.level in [5, 9]:
                    if not self.inverted_gravity:
                        self.inverted_gravity = True
                    self.ax.set_title(f'Level {self.level}: Inverted gravity for {mission_time:.0f}s')
                else:
                    self.ax.set_title(f'Level {self.level}: Stay at the path for {mission_time:.0f}s')
                if self.start_time == None:
                    self.start_time = self.time
                elapsed = self.time - self.start_time
                remaining = max(0.0, mission_time - elapsed)
                self.mission_text.set_text(f"Time left: {remaining:.1f}s")
                if elapsed >= mission_time:
                    self.mission_completed = True
                    self.inverted_gravity = False
            elif self.level in [2, 6]:
                mission_stars = {2: 3, 6: 5}[self.level]
                self.ax.set_title(f'Level {self.level}: Collect {mission_stars} stars')
                if self.start_time == None:
                    self.start_time = self.time
                active_stars = [i for i, (sx, _) in enumerate(self.objects) if i not in self.collected and sx > self.time - self.window_size_x]
                if self.time - self.last_obj_time > self.obj_interval and len(active_stars) < self.max_active_obj:
                    self.generate_star()
                    self.last_obj_time = self.time
                for i, (sx, sy) in enumerate(self.objects):
                    if i in self.collected:
                        continue
                    dx = self.player_x - sx
                    dy = self.player_y - sy
                    if abs(dx) < self.obj_radius_x and abs(dy) < self.obj_radius_y:
                        self.collected.add(i)
                        self.increment_score(10)
                        self.obj_counter += 1
                        self.plot_objects[i].set_visible(False)
                        self.get_logger().info(f"Star collected!")
                self.mission_text.set_text(f"Stars collected: {self.obj_counter}")
                if self.obj_counter >= mission_stars:
                    self.mission_completed = True
                    self.obj_counter = 0
                    self.clear_objects()
            elif self.level in [3, 8]:
                mission_asteroids = {3: 4, 8: 6}[self.level]
                self.ax.set_title(f'Level {self.level}: Avoid {mission_asteroids} asteroids')
                if self.start_time == None:
                    self.start_time = self.time
                active_asteroids = [i for i, (sx, _) in enumerate(self.objects) if i not in self.collected and sx > self.time - self.window_size_x]
                if self.time - self.last_obj_time > self.obj_interval and len(active_asteroids) < self.max_active_obj:
                    self.generate_asteroid()
                    self.last_obj_time = self.time
                for i, (sx, sy) in enumerate(self.objects):
                    if i in self.collected:
                        continue
                    dx = self.player_x - sx
                    dy = self.player_y - sy
                    if abs(dx) < self.obj_radius_x and abs(dy) < self.obj_radius_y:
                        self.collected.add(i)
                        self.decrement_score(5)
                        self.plot_objects[i].set_visible(False)
                        self.get_logger().info(f"Hit asteroid!")
                    elif abs(dx) < self.obj_radius_x and abs(dy) > self.obj_radius_y:
                        self.collected.add(i)
                        self.increment_score(10)
                        self.obj_counter += 1
                        self.get_logger().info(f"Asteroid avoided!")
                self.mission_text.set_text(f"Asteroids avoided: {self.obj_counter}")
                if self.obj_counter >= mission_asteroids:
                    self.mission_completed = True
                    self.obj_counter = 0
                    self.clear_objects()
            else:
                self.mission_text.set_text(f"")
                if (self.level < 10):
                    self.level += 1
                    self.update_level_effects()
            
            if self.mission_completed:
                self.mission_completed = False
                self.start_time = None
                self.increment_score(30)
                self.level += 1
                self.update_level_effects()

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
            color = (0, 0, 0) # black == safe
        elif norm_dist < 0.7:
            color = (0.5, 0.5, 0.5) # grey == careful
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