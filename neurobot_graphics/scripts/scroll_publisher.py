#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import tkinter as tk
from tkinter import ttk
import sys
import csv
import os

class ScrollPublisherNode(Node):
    def __init__(self, freq=0.5, ampl=1.0, disturb=0.0, level=1):
        super().__init__('scroll_publisher_node')

        self.slider_publisher_ = self.create_publisher(
            Float32MultiArray,
            'SliderParameters',
            10
        )

        self.game_publisher_ = self.create_publisher(
            Int32,
            'GameParameters',
            10
        )

        self.freq = freq
        self.ampl = ampl
        self.disturb = disturb
        self.duration = 0.5
        self.interval = 30.0
        self.level = level

        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        slider_msg = Float32MultiArray()
        slider_msg.data = [self.freq, self.ampl, self.disturb, self.duration, self.interval]
        self.slider_publisher_.publish(slider_msg)

        self.get_logger().info(f"Publishing [F{self.freq:.2f} A{self.ampl:.2f} D{self.disturb:.2f} d{self.duration:.2f} i{self.interval:.2f}]")
    
class ScrollGUI:
    def __init__(self, node):
        self.node = node

        self.frequency = node.freq
        self.amplitude = node.ampl
        self.disturbance = node.disturb
        self.duration = node.duration
        self.interval = node.interval

        self.window = tk.Tk()
        self.window.title("Gaming Parameters Controller")
        self.window.geometry("1000x700")

        frequency_frame = ttk.Frame(self.window)
        frequency_frame.pack(pady=10)

        self.frequency_scroller = ttk.Scale(frequency_frame, from_=0.0, to=10.0, orient="horizontal", command=self.update_frequency, length=400)
        self.frequency_scroller.set(self.frequency)
        self.frequency_scroller.pack(pady=5)

        self.frequency_label = tk.Label(frequency_frame, text=f"Frequency: {self.frequency:.2f} Hz", font=("Arial", 14))
        self.frequency_label.pack(pady=2)

        amplitude_frame = ttk.Frame(self.window)
        amplitude_frame.pack(pady=10)

        self.amplitude_scroller = ttk.Scale(amplitude_frame, from_=0.0, to=10.0, orient="horizontal", command=self.update_amplitude, length=400)
        self.amplitude_scroller.set(self.amplitude)
        self.amplitude_scroller.pack(pady=5)

        self.amplitude_label = tk.Label(amplitude_frame, text=f"Amplitude: {self.amplitude:.2f}", font=("Arial", 14))
        self.amplitude_label.pack(pady=2)

        disturbance_frame = ttk.Frame(self.window)
        disturbance_frame.pack(pady=10)

        self.disturbance_scroller = ttk.Scale(disturbance_frame, from_=0.0, to=1.0, orient="horizontal", command=self.update_disturbance, length=400)
        self.disturbance_scroller.set(self.disturbance)
        self.disturbance_scroller.pack(pady=5)

        self.disturbance_label = tk.Label(disturbance_frame, text=f"Disturbance: {self.disturbance:.2f}", font=("Arial", 14))
        self.disturbance_label.pack(pady=2)

        self.duration_scroller = ttk.Scale(disturbance_frame, from_=0.5, to=5.0, orient="horizontal", command=self.update_duration, length=400)
        self.duration_scroller.set(self.duration)
        self.duration_scroller.pack(pady=5)

        self.duration_label = tk.Label(disturbance_frame, text=f"Disturbance duration: {self.duration:.2f} s", font=("Arial", 14))
        self.duration_label.pack(pady=2)

        self.interval_scroller = ttk.Scale(disturbance_frame, from_=5.0, to=30.0, orient="horizontal", command=self.update_interval, length=400)
        self.interval_scroller.set(self.interval)
        self.interval_scroller.pack(pady=5)

        self.interval_label = tk.Label(disturbance_frame, text=f"Interval between disturbances: {self.interval:.2f} s", font=("Arial", 14))
        self.interval_label.pack(pady=2)

        self.update_signal_button = tk.Button(self.window, text="Update signal", command=self.update_signal, font=("Arial", 14), width=20, height=2, bg="green", fg="white")
        self.update_signal_button.pack(pady=10)

        level_frame = ttk.Frame(self.window)
        level_frame.pack(pady=10)

        self.level_label = tk.Label(level_frame, text="Select Level", font=("Arial", 14))
        self.level_label.pack(pady=5)

        self.level_var = tk.StringVar()
        self.level_combobox = ttk.Combobox(level_frame, textvariable=self.level_var, font=("Arial", 14), state="readonly", width=10)
        self.level_combobox['values'] = [str(i) for i in range(1, 11)]
        self.level_combobox.set(str(self.node.level))
        self.level_combobox.pack(pady=2)

        self.update_level_button = tk.Button(self.window, text="Update level", command=self.update_level, font=("Arial", 14), width=20, height=2, bg="blue", fg="white")
        self.update_level_button.pack(pady=10)

        self.exit_button = tk.Button(self.window, text="Salir", command=self.close, font=("Arial", 14), width=20, height=2, bg="red", fg="white")
        self.exit_button.pack(pady=10)
    
    def update_frequency(self, val):
        self.frequency = float(val)
        self.frequency_label.config(text=f"Current Frequency: {self.frequency:.2f} Hz")
    
    def update_amplitude(self, val):
        self.amplitude = float(val)
        self.amplitude_label.config(text=f"Current Amplitude: {self.amplitude:.2f}")
    
    def update_disturbance(self, val):
        self.disturbance = float(val)
        self.disturbance_label.config(text=f"Current Disturbance: {self.disturbance:.2f}")

    def update_duration(self, val):
        self.duration = float(val)
        self.duration_label.config(text=f"Disturbance duration: {self.duration:.2f} s")
    
    def update_interval(self, val):
        self.interval = float(val)
        self.interval_label.config(text=f"Interval between disturbances: {self.interval:.2f} s")
    
    def update_signal(self):
        self.node.freq = self.frequency
        self.node.ampl = self.amplitude
        self.node.disturb = self.disturbance
        self.node.duration = self.duration
        self.node.interval = self.interval
    
    def update_level(self):
        self.level = int(self.level_var.get())

        game_msg = Int32()
        game_msg.data = self.level
        self.node.game_publisher_.publish(game_msg)

        self.node.get_logger().info(f"Publishing [L{self.level}]")
    
    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.window.destroy()
    
    def run(self):
        self.window.after(10, self.spin_once)
        self.window.mainloop()
    
    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.window.after(10, self.spin_once)

def load_values(file_path):
    if not os.path.isfile(file_path):
        print(f"File '{file_path}' does not exist")
        return 0.5, 1.0, 0.0, 1
    
    try:
        with open(file_path, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            if not rows:
                return 0.5, 1.0, 0.0, 1
            last_row = rows[-1]
            freq = float(last_row.get("F", 0.5))
            ampl = float(last_row.get("A", 1.0))
            disturb = float(last_row.get("D", 0.0))
            level = int(last_row.get("L", 1))
            return freq, ampl, disturb, level
    except Exception as e:
        print(f"Error reading file {e}")
        return 0.5, 1.0, 0.0, 1

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
        freq, ampl, disturb, level = load_values(csv_path)
    else:
        freq, ampl, disturb, level = 0.5, 1.0, 0.0, 1

    scroll_publisher_node = ScrollPublisherNode(freq, ampl, disturb, level)

    gui = ScrollGUI(scroll_publisher_node)

    try:
        gui.run()
    except KeyboardInterrupt:
        gui.window.destroy()
    finally:
        scroll_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()