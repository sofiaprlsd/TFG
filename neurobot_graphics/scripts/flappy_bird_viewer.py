#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
import numpy as np
import sys
import csv
import os
from datetime import datetime

HOME_DIR = os.path.expanduser("~")
DIR = os.path.join(HOME_DIR, "database")
os.makedirs(DIR, exist_ok=True)

class FlappyBirdViewerNode(Node):
    def __init__(self):
        super().__init__('flappy_bird_viewer_node')

        self.signal_subscriber_ = self.create_subscription(
            Float32,
            'CleanSignal',
            self.signalcallback,
            10
        )

        self.disturbance_subscriber_ = self.create_subscription(
            Float32,
            'Disturbance',
            self.disturbcallback,
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

        self.reference_subscriber_ = self.create_subscription(
            Float32MultiArray,
            'MotorParameters',
            self.referencescallback,
            10
        )
        
        self.time = 0.0
        self.window_size_x = 2
        self.window_size_y = 10
        self.sample_amount = 500
        self.player_x = 0.25
        self.player_y = 0.0
        self.offset_y = 3.0
        self.traj_time = []
        self.error = []
        self.mean_error = 0.0
        self.player_active = False
        self.last_disturb_val = 0.0
        self.assist_level = 0

        self.log_time = []
        self.log_signal = []
        self.log_upper_limit = []
        self.log_lower_limit = []
        self.log_disturb = []
        self.log_player_x = []
        self.log_player_y = []
        self.log_offset_y = []
        self.log_error = []
        self.log_collision = []
        self.log_assistance = []

        self.time_data = np.zeros(self.sample_amount)
        self.signal_data = np.zeros(self.sample_amount)
        self.signal_upper = np.zeros(self.sample_amount)
        self.signal_lower = np.zeros(self.sample_amount)
        self.disturb_data = np.zeros(self.sample_amount)

        plt.ion()   # Update graph in real time
        self.fig, (self.ax, self.ax_traj) = plt.subplots(2, 1, figsize=(10, 10), sharex=False)

        self.ax.set_facecolor('black')
        self.ax.set_title('Flappy Bird Viewer - Waiting player...')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Amplitude')
        
        self.line, = self.ax.plot([], [], color='blue', label='Signal')
        self.line_upper, = self.ax.plot([], [], linestyle='--', color='grey', label='Signal + offset')
        self.line_lower, = self.ax.plot([], [], linestyle='--', color='grey', label='Signal - offset')
        self.line_disturb, = self.ax.plot(self.time_data, self.disturb_data, color='green', label='Disturbance')
        self.player, = self.ax.plot([], [], 'ro', label='Player')
        
        self.ax.set_xlim(0, self.window_size_x)
        self.ax.set_ylim(-self.window_size_y, self.window_size_y)

        self.ax_traj.set_facecolor('black')
        self.ax_traj.set_title('Trajectory Error')
        self.ax_traj.set_xlabel('Time (s)')
        self.ax_traj.set_ylabel('Error')
        self.traj_line, = self.ax_traj.plot([], [], color='red', label='Error')

        # Show window
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def signalcallback(self, msg):
        if not self.player_active:
            return
        
        self.ax.set_facecolor('white')
        self.ax.set_title(f'Flappy Bird Viewer | Offset: {self.offset_y:.2f}')
        self.ax_traj.set_facecolor('white')
        self.ax_traj.set_title(f'Trajectory Error | Mean: {self.mean_error:.3f}')
        
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
        
        # Check if the player is colliding with any limit (signals)
        upper_limit = self.signal_upper[-1]
        lower_limit = self.signal_lower[-1]
        if not (lower_limit < self.player_y < upper_limit):
            self.get_logger().info("Collision!!!")
        
        if self.time_data[-1] > self.window_size_x:
            self.ax.set_xlim(self.time - self.window_size_x, self.time)
        
        self.traj_time.append(self.time)
        if len(self.traj_time) > self.sample_amount:
            self.traj_time = self.traj_time[-self.sample_amount:]
        
        index = np.argmin(np.abs(self.time_data - self.player_x))
        ref_y = self.signal_data[index]
        self.error.append(abs(self.player_y - ref_y))
        if len(self.error) > self.sample_amount:
            self.error = self.error[-self.sample_amount:]

        # Plot sigal and player position
        self.line.set_xdata(self.time_data)
        self.line.set_ydata(self.signal_data)
        self.line_upper.set_xdata(self.time_data)
        self.line_upper.set_ydata(self.signal_upper)
        self.line_lower.set_xdata(self.time_data)
        self.line_lower.set_ydata(self.signal_lower)
        # If disturbance is different from 0
        if np.any(self.disturb_data):
            self.line_disturb.set_xdata(self.time_data)
            self.line_disturb.set_ydata(self.disturb_data)
            self.line_disturb.set_visible(True)
        else:
            self.line_disturb.set_visible(False)
        self.player.set_xdata(self.player_x)
        self.player.set_ydata(self.player_y)
        
        error_len = len(self.error)
        error_time = self.traj_time[-error_len:]
        if error_len > 0:
            self.traj_line.set_xdata(error_time)
            self.traj_line.set_ydata(self.error)
            self.mean_error = np.mean(self.error)
        else:
            self.traj_line.set_xdata([])
            self.traj_line.set_ydata([])

        # Adjust graph to the updated data
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax_traj.relim()
        self.ax_traj.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        self.log_time.append(self.time)
        self.log_signal.append(msg.data)
        self.log_upper_limit.append(msg.data + self.offset_y)
        self.log_lower_limit.append(msg.data - self.offset_y)
        self.log_disturb.append(self.last_disturb_val)
        self.log_player_x.append(self.player_x)
        self.log_player_y.append(self.player_y)
        self.log_offset_y.append(self.offset_y)
        self.log_error.append(abs(self.player_y - msg.data))
        self.log_collision.append(int(not (self.signal_lower[-1] < self.player_y < self.signal_upper[-1])))
        self.log_assistance.append(self.assist_level)
    
    def disturbcallback(self, msg):
        self.last_disturb_val = msg.data
    
    def offsetcallback(self, msg):
        self.offset_y = msg.data[2]
    
    def positioncallback(self, msg):
        self.player_y = msg.data
    
    def referencescallback(self, msg):
        self.player_x = msg.data[2]
        self.time = msg.data[3]
        self.assist_level = int(msg.data[4])
        self.player_active = True

def savemetrics(node, patient_id):
    now = datetime.now()
    date = now.strftime("%Y-%m-%d")
    index = 1
    ext = ".csv"
    ID_DIR = os.path.join(DIR, patient_id)
    os.makedirs(ID_DIR, exist_ok=True)
    METRICS_DIR = os.path.join(ID_DIR, "metrics")
    os.makedirs(METRICS_DIR, exist_ok=True)

    while True:
        file_name = f"{patient_id}-{date}-metrics_{index}{ext}"
        file_path = os.path.join(METRICS_DIR, file_name)
        if not os.path.exists(file_path):
            break
        index += 1

    header = ["time", "signal", "upper_limit", "lower_limit", "disturbance", "player_x", "player_y", "offset_y", "error", "collision", "assistance"]

    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        for i in range(len(node.log_time)):
            writer.writerow([
                node.log_time[i],
                node.log_signal[i],
                node.log_upper_limit[i],
                node.log_lower_limit[i],
                node.log_disturb[i],
                node.log_player_x[i],
                node.log_player_y[i],
                node.log_offset_y[i],
                node.log_error[i],
                node.log_collision[i],
                node.log_assistance[i]
            ])

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print(f"Error: Especifique archivo con los datos del paciente")
        sys.exit(1)
    
    csv_path = sys.argv[1]
    full_path = os.path.expanduser(csv_path)
    if not os.path.isfile(full_path):
        print(f"Error: El archivo no existe")
        sys.exit(1)
    patient_id = os.path.basename(os.path.dirname(full_path))
    
    flappy_bird_viewer_node = FlappyBirdViewerNode()

    try:
        rclpy.spin(flappy_bird_viewer_node)
    except KeyboardInterrupt:
        pass
    finally:
        savemetrics(flappy_bird_viewer_node, patient_id)
        plt.close('all')
        flappy_bird_viewer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()