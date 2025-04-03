#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk

class ScrollPublisherNode(Node):
    def __init__(self):
        super().__init__('scroll_publisher_node')

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'SliderParameters',
            10
        )

        self.freq = 0.5
        self.ampl = 1.0

        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        msg = Float32MultiArray()
        msg.data = [self.freq, self.ampl]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing F{self.freq:.2f} A{self.ampl:.2f}")
    
class ScrollGUI:
    def __init__(self, node):
        self.node = node

        self.frequency = 0.5
        self.amplitude = 1.0

        self.window = tk.Tk()
        self.window.title("Frequency and Amplitude controller")
        self.window.geometry("600x400")

        self.frequency_label = tk.Label(self.window, text="Frequency (Hz)", font=("Arial", 14))
        self.frequency_label.pack(pady=5)

        self.frequency_scroller = ttk.Scale(self.window, from_=0.0, to=10.0, orient="horizontal", command=self.update_frequency, length=400)
        self.frequency_scroller.set(self.frequency)
        self.frequency_scroller.pack(pady=5)

        self.frequency_label = tk.Label(self.window, text=f"Frequency: {self.frequency:.2f} Hz", font=("Arial", 14))
        self.frequency_label.pack(pady=5)

        self.amplitude_label = tk.Label(self.window, text="Amplitude", font=("Arial", 14))
        self.amplitude_label.pack(pady=5)

        self.amplitude_scroller = ttk.Scale(self.window, from_=0.0, to=10.0, orient="horizontal", command=self.update_amplitude, length=400)
        self.amplitude_scroller.set(self.amplitude)
        self.amplitude_scroller.pack(pady=5)

        self.amplitude_label = tk.Label(self.window, text=f"Amplitude: {self.amplitude:.2f}", font=("Arial", 14))
        self.amplitude_label.pack(pady=5)

        self.update_button = tk.Button(self.window, text="Update values", command=self.update_values, font=("Arial", 14), width=20, height=2, bg="green", fg="white")
        self.update_button.pack(pady=10)

        self.update_button = tk.Button(self.window, text="Salir", command=self.close, font=("Arial", 14), width=20, height=2, bg="red", fg="white")
        self.update_button.pack(pady=10)
    
    def update_frequency(self, val):
        self.frequency = float(val)
        self.frequency_label.config(text=f"Frequency: {self.frequency:.2f} Hz")
    
    def update_amplitude(self, val):
        self.amplitude = float(val)
        self.amplitude_label.config(text=f"Amplitude: {self.amplitude:.2f}")
    
    def update_values(self):
        self.node.freq = self.frequency
        self.node.ampl = self.amplitude
    
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

def main(args=None):
    rclpy.init(args=args)
    scroll_publisher_node = ScrollPublisherNode()

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