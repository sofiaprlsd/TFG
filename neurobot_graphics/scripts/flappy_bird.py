#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
from pynput import keyboard
import pygame

pygame.mixer.init()
level_up_sound = pygame.mixer.Sound('../sounds/level_up.wav')
star_gained_sound = pygame.mixer.Sound('../sounds/star_gained.wav')
asteroid_hit_sound = pygame.mixer.Sound('../sounds/asteroid_hit.wav')

class FlappyBirdNode(Node):
    def __init__(self):
        super().__init__('flappy_bird_node')

        self.signal_subscriber_ = self.create_subscription(
            Float32,
            'CleanSignal',
            self.listenercallback,
            10
        )

        self.disturbance_subscriber_ = self.create_subscription(
            Float32,
            'Disturbance',
            self.disturbcallback,
            10
        )

        self.level_subscriber_ = self.create_subscription(
            Int32MultiArray,
            'GameParameters',
            self.levelcallback,
            10
        )

        self.offset_subscriber_ = self.create_subscription(
            Float32MultiArray,
            'SliderParameters',
            self.offsetcallback,
            10
        )

        self.position_subscriber_ = self.create_subscription(
            Float32,
            'ActuatorPosition',
            self.positioncallback,
            10
        )

        self.references_publisher_ = self.create_publisher(
            Float32MultiArray,
            'MotorParameters',
            10
        )

        # self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        # self.keyboard_listener.start()

        self.time = 0.0
        self.window_size_x = 2
        self.window_size_y = 3
        self.sample_amount = 500
        self.player_x = 0.25
        self.player_y = 0.0
        self.offset_y = 3.0
        self.max_offset = 3.0
        self.min_offset = 1.0
        self.level = 1
        self.max_level = 10
        self.assistance = 5
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
        self.last_disturb_val = 0.0
        self.disturb_markers = []
        self.disturb_type = 1.0
        self.disturb_markers_active = []
        self.direction = 1.0

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
        self.disturb_data = np.zeros(self.sample_amount)

        plt.ion()   # Update graph in real time
        self.fig, self.ax = plt.subplots(figsize=(12, 8))

        self.score_text = self.ax.text(0.95, 0.9, f"Score{self.score}", transform=self.ax.transAxes, fontsize=12, color='black', ha='right')
        self.mission_text = self.ax.text(0.05, 0.9, "", transform=self.ax.transAxes, fontsize=12, color='black', ha='left')

        self.line, = self.ax.plot(self.time_data, self.signal_data, color='lightblue', label='Signal')
        self.line_upper, = self.ax.plot(self.time_data, self.signal_upper, linestyle='--', color='white', label='Signal + offset')
        self.line_lower, = self.ax.plot(self.time_data, self.signal_lower, linestyle='--', color='white', label='Signal - offset')
        self.line_disturb, = self.ax.plot(self.time_data, self.disturb_data, color='orange', label='Disturbance')
        self.player, = self.ax.plot(self.player_x, self.player_y, 'ro', label='Player')
        
        self.ax.set_xlim(0, self.window_size_x)
        self.ax.set_ylim(-self.window_size_y, self.window_size_y)
        # Do not show axis numbers
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.updatelevel()

    def updatelevel(self):
        colors = self.level_colors[(self.level - 1) % len(self.level_colors)]
        self.ax.set_facecolor(colors['bg'])
        self.line.set_color(colors['line'])
        self.ax.set_title(f'Flappy Bird Level {self.level}')
        level_up_sound.play()
        self.fig.canvas.draw()
        self.assistance = max(0, self.assistance - (self.level - 1) // 2) # each 2 game levels decrement 1 level of assistance
        self.get_logger().info(f"Upgrade to level {self.level} with assistance {self.assistance}")
    
    def incrementscore(self, points):
        self.score += points
        self.score_text.set_text(f"Score: {self.score}")
    
    def decrementscore(self, points):
        self.score -= points
        if self.score < 0:
            self.score = 0
        self.score_text.set_text(f"Score: {self.score}")
    
    def clearobjects(self):
        for s in self.plot_objects:
            s.remove()
        self.objects = []
        self.plot_objects = []
        self.collected = set()
    
    def generatestar(self):
        x = self.time + np.random.uniform(0.5, 0.4)
        y = self.signal_data[-1]
        self.objects.append([x, y])
        o = self.ax.plot(x, y, marker="*", color="gold", markersize=25)[0]
        self.plot_objects.append(o)
    
    def generateasteroid(self):
        x = self.time + np.random.uniform(0.5, 4.0)
        y = np.random.uniform(0.3, 1.5)
        if np.random.rand() > 0.5:
            y = np.random.uniform(-1.5, -0.3)
        self.objects.append([x, y])
        o = self.ax.plot(x, y, marker="o", color="brown", markersize=25)[0]
        self.plot_objects.append(o)
    
    def generatedisturb(self):
        # Detect change from 0 to another value or viceversa
        if self.disturb_data[-1] != 0.0 and self.disturb_data[-2] == 0.0:
            t = self.time
            y = self.signal_data[-1]
            color = 'orange' if self.disturb_type == 2.0 else 'green'
            marker_type = '^' if self.last_disturb_val > 0 else 'v'
            triangle = self.ax.plot(t, y, marker=marker_type, color=color, markersize=20)[0]
            self.disturb_markers.append((t, y, triangle))
    
    def on_press(self, key):
        try:
            direction = -1 if self.inverted_gravity else 1
            if key == keyboard.Key.up:
                self.player_y += 0.1 * direction
            elif key == keyboard.Key.down:
                self.player_y -= 0.1 * direction
        except:
            pass
    
    def listenercallback(self, msg):
        self.time_data = np.roll(self.time_data, -1)
        self.signal_data = np.roll(self.signal_data, -1)
        self.signal_upper = np.roll(self.signal_upper, -1)
        self.signal_lower = np.roll(self.signal_lower, -1)
        self.disturb_data = np.roll(self.disturb_data, -1)

        self.time_data[-1] = self.time
        self.signal_data[-1] = msg.data
        self.signal_upper[-1] = msg.data + self.offset_y
        self.signal_lower[-1] = msg.data - self.offset_y
        self.disturb_data[-1] = self.last_disturb_val

        upper_limit = self.signal_upper[-1]
        lower_limit = self.signal_lower[-1]

        # If there is a disturbance advaise it to the player
        self.generatedisturb()

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
                    self.generatestar()
                    self.last_obj_time = self.time
                for i, (sx, sy) in enumerate(self.objects):
                    if i in self.collected:
                        continue
                    dx = self.player_x - sx
                    dy = self.player_y - sy
                    if abs(dx) < self.obj_radius_x and abs(dy) < self.obj_radius_y:
                        self.collected.add(i)
                        star_gained_sound.play()
                        self.incrementscore(10)
                        self.obj_counter += 1
                        self.plot_objects[i].set_visible(False)
                        self.get_logger().info(f"Star collected!")
                self.mission_text.set_text(f"Stars collected: {self.obj_counter}")
                if self.obj_counter >= mission_stars:
                    self.mission_completed = True
                    self.obj_counter = 0
                    self.clearobjects()
            elif self.level in [3, 8]:
                mission_asteroids = {3: 4, 8: 6}[self.level]
                self.ax.set_title(f'Level {self.level}: Avoid {mission_asteroids} asteroids')
                if self.start_time == None:
                    self.start_time = self.time
                active_asteroids = [i for i, (sx, _) in enumerate(self.objects) if i not in self.collected and sx > self.time - self.window_size_x]
                if self.time - self.last_obj_time > self.obj_interval and len(active_asteroids) < self.max_active_obj:
                    self.generateasteroid()
                    self.last_obj_time = self.time
                for i, (sx, sy) in enumerate(self.objects):
                    if i in self.collected:
                        continue
                    dx = self.player_x - sx
                    dy = self.player_y - sy
                    if abs(dx) < self.obj_radius_x and abs(dy) < self.obj_radius_y:
                        self.collected.add(i)
                        asteroid_hit_sound.play()
                        self.decrementscore(5)
                        self.plot_objects[i].set_visible(False)
                        self.get_logger().info(f"Hit asteroid!")
                    elif abs(dx) < self.obj_radius_x and abs(dy) > self.obj_radius_y:
                        self.collected.add(i)
                        self.incrementscore(10)
                        self.obj_counter += 1
                        self.get_logger().info(f"Asteroid avoided!")
                self.mission_text.set_text(f"Asteroids avoided: {self.obj_counter}")
                if self.obj_counter >= mission_asteroids:
                    self.mission_completed = True
                    self.obj_counter = 0
                    self.clearobjects()
            else:
                self.mission_text.set_text(f"")
                if (self.level < 10):
                    self.level += 1
                    self.updatelevel()
            
            if self.mission_completed:
                self.mission_completed = False
                self.start_time = None
                self.incrementscore(30)
                self.level += 1
                self.updatelevel()

        # When time is greater than window size, 
        # increment player_x to stay in same spot of the window
        self.time += 0.01
        if self.time >= self.window_size_x:
            self.player_x += 0.01

        if self.time_data[-1] > self.window_size_x:
            self.ax.set_xlim(self.time - self.window_size_x, self.time)

        # Plot sigal, disturbance and player position
        self.line.set_xdata(self.time_data)
        self.line.set_ydata(self.signal_data)
        self.line_upper.set_xdata(self.time_data)
        self.line_upper.set_ydata(self.signal_upper)
        self.line_lower.set_xdata(self.time_data)
        self.line_lower.set_ydata(self.signal_lower)
        self.player.set_xdata(self.player_x)
        self.player.set_ydata(self.player_y)

        visible_marker_time = self.time - self.window_size_x
        reachable_markers = []
        for x, y, marker in self.disturb_markers:
            if x < visible_marker_time:
                marker.remove()
            else:
                reachable_markers.append((x, y, marker))
        self.disturb_markers = reachable_markers

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

        # Publish signal and disturbance 'y' references when closest to player_x
        # Publish time to sync with viewer
        index = np.argmin(np.abs(self.time_data - self.player_x))
        y_signal = self.signal_data[index] * self.direction
        y_disturb = self.disturb_data[index] * self.direction
        signal_msg = Float32MultiArray()
        signal_msg.data = [y_signal, y_disturb, self.player_x, self.time, float(self.assistance)]
        self.references_publisher_.publish(signal_msg)
    
    def disturbcallback(self, msg):
        self.last_disturb_val = msg.data
    
    def levelcallback(self, msg):
        self.level = msg.data[0]
        self.assistance = msg.data[1]
        self.mission_completed = False
        self.game_completed = False
        self.inside_limits = True
        self.start_time = None
        self.direction = 1.0
        self.clearobjects()
        self.updatelevel()

        if self.level in [2, 6]:
            self.obj_counter = 0
            self.generatestar()
            self.last_obj_time = self.time
        elif self.level in [5, 9]:
            self.direction = -1.0
    
    def offsetcallback(self, msg):
        self.offset_y = msg.data[2]
        self.disturb_type = msg.data[7]
    
    def positioncallback(self, msg):
        direction = -1 if self.inverted_gravity else 1
        self.player_y = msg.data * direction

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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()