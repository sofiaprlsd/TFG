#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import sys
import csv
import os
from datetime import datetime

HOME_DIR = os.path.expanduser("~")
DIR = os.path.join(HOME_DIR, "database")
os.makedirs(DIR, exist_ok=True)

# ----------------------- ROS2 NODE ----------------------- #
class ScrollPublisherNode(Node):
    def __init__(self, freq=0.5, ampl=1.0, disturb=0.0, duration=0.5, period=30.0, level=1, assist = 0):
        super().__init__('scroll_publisher_node')

        self.slider_publisher_ = self.create_publisher(
            Float32MultiArray,
            'SliderParameters',
            10
        )

        self.game_publisher_ = self.create_publisher(
            Int32MultiArray,
            'GameParameters',
            10
        )
        
        self.freq = freq
        self.ampl = ampl
        self.offset = 3.0
        self.signal = "sinusoidal"
        self.disturb = disturb
        self.duration = duration
        self.period = period
        self.disturb_signal = "sinusoidal"
        self.level = level
        self.assist = assist

        self.timer = self.create_timer(0.1, self.publishdata)

    def publishdata(self):
        signal = 1.0
        disturb_signal = 1.0
        if self.signal in ["sinusoidal", "step"]:
            signal = {"sinusoidal": 1.0, "step": 2.0}[self.signal]
        if self.disturb_signal in ["sinusoidal", "step"]:
            disturb_signal = {"sinusoidal": 1.0, "step": 2.0}[self.disturb_signal]
        slider_msg = Float32MultiArray()
        slider_msg.data = [self.freq, self.ampl, self.offset, signal, self.disturb, self.duration, self.period, disturb_signal]
        self.slider_publisher_.publish(slider_msg)

        self.get_logger().info(f"Publishing [F{self.freq:.2f} A{self.ampl:.2f} O{self.offset:.2f} T{signal} D{self.disturb:.2f} d{self.duration:.2f} p{self.period:.2f} t{disturb_signal}]")

# ----------------------- GUI GAMING ----------------------- #
class ScrollGUI:
    def __init__(self, node, patient_id):
        self.node = node
        self.signal_var = "sinusoidal"
        self.disturb_signal_var = "sinusoidal"
        self.assistance = 1

        now = datetime.now()
        date = now.strftime("%Y-%m-%d")
        index = 1
        ext = ".csv"
        ID_DIR = os.path.join(DIR, patient_id)
        os.makedirs(ID_DIR, exist_ok=True)
        CONFIG_DIR = os.path.join(ID_DIR, "config")
        os.makedirs(CONFIG_DIR, exist_ok=True)

        header = ["frequency", "amplitude", "offset", "signal", "disturbance", "duration", "period", "mode"]

        while True:
            file_name = f"{patient_id}-{date}-config_{index}{ext}"
            file_path = os.path.join(CONFIG_DIR, file_name)
            if not os.path.exists(file_path):
                with open(file_path, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(header)
                break
            index += 1
        
        self.file_path = file_path
        self.frequency = node.freq
        self.amplitude = node.ampl
        self.offset = node.offset
        self.disturbance = node.disturb
        self.duration = node.duration
        self.period = node.period
        self.level = node.level

        self.window = tk.Tk()
        self.window.title("Gaming Parameters Controller")
        self.window.geometry("1000x900")

        signal_frame = ttk.Frame(self.window)
        signal_frame.pack(pady=10)

        self.frequency_label = tk.Label(signal_frame, text=f"Signal Frequency: {self.frequency:.2f} Hz", font=("Arial", 14))
        self.frequency_label.pack(pady=2)

        self.frequency_scroller = ttk.Scale(signal_frame, from_=0.0, to=3.0, orient="horizontal", command=self.updatefrequency, length=400)
        self.frequency_scroller.set(self.frequency)
        self.frequency_scroller.pack(pady=5)

        self.amplitude_label = tk.Label(signal_frame, text=f"Signal Amplitude: {self.amplitude:.2f}", font=("Arial", 14))
        self.amplitude_label.pack(pady=2)

        self.amplitude_scroller = ttk.Scale(signal_frame, from_=0.0, to=2.0, orient="horizontal", command=self.updateamplitude, length=400)
        self.amplitude_scroller.set(self.amplitude)
        self.amplitude_scroller.pack(pady=5)

        self.offset_label = tk.Label(signal_frame, text=f"Signal Offset: {self.offset:.2f}", font=("Arial", 14))
        self.offset_label.pack(pady=2)

        self.offset_scroller = ttk.Scale(signal_frame, from_=1.0, to=5.0, orient="horizontal", command=self.updateoffset, length=400)
        self.offset_scroller.set(self.offset)
        self.offset_scroller.pack(pady=5)

        self.signal_var = tk.StringVar(value="sinusoidal")
        tk.Label(self.window, text="Signal").pack()
        ttk.Combobox(self.window, textvariable=self.signal_var, values=["sinusoidal", "step"], state="readonly").pack()

        disturbance_frame = ttk.Frame(self.window)
        disturbance_frame.pack(pady=10)

        self.disturbance_label = tk.Label(disturbance_frame, text=f"Disturbance Amplitude: {self.disturbance:.2f}", font=("Arial", 14))
        self.disturbance_label.pack(pady=2)

        self.disturbance_scroller = ttk.Scale(disturbance_frame, from_=0.0, to=2.0, orient="horizontal", command=self.updatedisturbance, length=400)
        self.disturbance_scroller.set(self.disturbance)
        self.disturbance_scroller.pack(pady=5)

        self.duration_label = tk.Label(disturbance_frame, text=f"Disturbance Duration: {self.duration:.2f} s", font=("Arial", 14))
        self.duration_label.pack(pady=2)

        self.duration_scroller = ttk.Scale(disturbance_frame, from_=0.1, to=5.0, orient="horizontal", command=self.updateduration, length=400)
        self.duration_scroller.set(self.duration)
        self.duration_scroller.pack(pady=5)

        self.period_label = tk.Label(disturbance_frame, text=f"Disturbance Period: {self.period:.2f} s", font=("Arial", 14))
        self.period_label.pack(pady=2)

        self.period_scroller = ttk.Scale(disturbance_frame, from_=5.0, to=30.0, orient="horizontal", command=self.updateperiod, length=400)
        self.period_scroller.set(self.period)
        self.period_scroller.pack(pady=5)

        self.disturb_signal_var = tk.StringVar(value="sinusoidal")
        tk.Label(self.window, text="Disturbance Signal").pack()
        ttk.Combobox(self.window, textvariable=self.disturb_signal_var, values=["sinusoidal", "step"], state="readonly").pack()

        self.update_signal_button = tk.Button(self.window, text="Update signal", command=self.updatesignal, font=("Arial", 14), width=20, height=2, bg="green", fg="white")
        self.update_signal_button.pack(pady=10)

        assist_frame = ttk.Frame(self.window)
        assist_frame.pack(pady=10)

        self.assist_label = tk.Label(assist_frame, text=f"Robot assistance", font=("Arial", 14))
        self.assist_label.pack(pady=5)

        self.assist_var = tk.StringVar()
        self.assist_combobox = ttk.Combobox(assist_frame, textvariable=self.assist_var, font=("Arial", 14), state="readonly", width=10)
        self.assist_combobox['values'] = [str(i) for i in range(0, 6)]
        self.assist_combobox.set(str(self.node.assist))
        self.assist_combobox.pack(pady=2)

        level_frame = ttk.Frame(self.window)
        level_frame.pack(pady=10)

        self.level_label = tk.Label(level_frame, text="Game level", font=("Arial", 14))
        self.level_label.pack(pady=5)

        self.level_var = tk.StringVar()
        self.level_combobox = ttk.Combobox(level_frame, textvariable=self.level_var, font=("Arial", 14), state="readonly", width=10)
        self.level_combobox['values'] = [str(i) for i in range(1, 11)]
        self.level_combobox.set(str(self.node.level))
        self.level_combobox.pack(pady=2)

        self.update_level_button = tk.Button(self.window, text="Update levels", command=self.updatelevel, font=("Arial", 14), width=20, height=2, bg="blue", fg="white")
        self.update_level_button.pack(pady=10)

        self.exit_button = tk.Button(self.window, text="Exit", command=self.close, font=("Arial", 14), width=20, height=2, bg="red", fg="white")
        self.exit_button.pack(pady=10)
    
    def saveconfig(self):
        with open(self.file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.frequency,
                self.amplitude,
                self.offset,
                self.signal_var.get(),
                self.disturbance,
                self.duration,
                self.period,
                self.disturb_signal_var.get()
            ])
    
    def updatefrequency(self, val):
        self.frequency = float(val)
        self.frequency_label.config(text=f"Signal Frequency: {self.frequency:.2f} Hz")
    
    def updateamplitude(self, val):
        self.amplitude = float(val)
        self.amplitude_label.config(text=f"Signal Amplitude: {self.amplitude:.2f}")
    
    def updateoffset(self, val):
        self.offset = float(val)
        self.offset_label.config(text=f"Signal Offset: {self.offset:.2f}")
    
    def updatedisturbance(self, val):
        self.disturbance = float(val)
        self.disturbance_label.config(text=f"Disturbance: {self.disturbance:.2f}")

    def updateduration(self, val):
        self.duration = float(val)
        self.duration_label.config(text=f"Disturbance Duration: {self.duration:.2f} s")
    
    def updateperiod(self, val):
        self.period = float(val)
        self.period_label.config(text=f"Disturbance Period: {self.period:.2f} s")
    
    def updatesignal(self):
        self.node.freq = self.frequency
        self.node.ampl = self.amplitude
        self.node.offset = self.offset
        self.node.disturb = self.disturbance
        self.node.duration = self.duration
        self.node.period = self.period
        self.node.signal = self.signal_var.get()
        self.node.disturb_signal = self.disturb_signal_var.get()
        self.saveconfig()
    
    def updatelevel(self):
        self.level = int(self.level_var.get())
        self.assistance = int(self.assist_var.get())
        game_msg = Int32MultiArray()
        game_msg.data = [(self.level), (self.assistance)]
        self.node.game_publisher_.publish(game_msg)
        self.node.get_logger().info(f"Publishing [L{self.level:.0f}, a{self.assistance:.0f}]")
    
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

# ----------------------- GUI MAIN ----------------------- #
def loadcsv(file_path):
    default_values = {
        "frequency": 0.5, "amplitude": 1.0, "offset": 3.0, "signal": "sinusoidal", 
        "disturbance": 0.0, "duration": 0.5, "period": 30.0, "mode": "sinusoidal",
        "assistance": 1, "level": 1
    }

    try:
        with open(file_path, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            if not rows:
                return (
                    default_values["frequency"],
                    default_values["amplitude"],
                    default_values["disturbance"],
                    default_values["duration"],
                    default_values["period"],
                    default_values["level"]
                )
            last_row = rows[-1]
            return (
                float(last_row.get("frequency", default_values["frequency"])),
                float(last_row.get("amplitude", default_values["amplitude"])),
                float(last_row.get("disturbance", default_values["disturbance"])),
                float(last_row.get("duration", default_values["duration"])),
                float(last_row.get("period", default_values["period"])),
                int(last_row.get("level", default_values["level"]))
            )
    except Exception as e:
        print(f"Error reading file {e}")
        return (
            default_values["frequency"],
            default_values["amplitude"],
            default_values["disturbance"],
            default_values["duration"],
            default_values["period"],
            default_values["level"]
        )

def main(args=None):
    if len(sys.argv) != 2:
        print(f"Error: Specify patient file")
        sys.exit(1)

    full_path = os.path.expanduser(sys.argv[1])
    if not os.path.isfile(full_path):
        print(f"Error: El archivo no existe")
        sys.exit(1)
    
    patient_id = os.path.basename(os.path.dirname(full_path))
    freq, ampl, disturb, duration, period, level = loadcsv(full_path)

    node = ScrollPublisherNode(freq, ampl, disturb, duration, period, level)
    gui = ScrollGUI(node, patient_id)

    try:
        gui.run()
    except KeyboardInterrupt:
        gui.window.destroy()
    finally:
        node.destroy_node()

# ----------------------- GUI CONFIGURATION ----------------------- #
def startgui(args=None):
    if len(sys.argv) != 2:
        print(f"Error: Specify patient file")
        sys.exit(1)
    
    limit_node = rclpy.create_node("limit_gui_node")
    limit_publisher_ = limit_node.create_publisher(
        Int32MultiArray,
        'RobotLimits',
        10
    )
    
    start_root = tk.Tk()
    start_root.title("Configure limits")
    start_root.geometry("1000x900")

    content_frame = ttk.Frame(start_root)
    content_frame.pack(expand=True)

    min_published = tk.BooleanVar(value=False)
    max_published = tk.BooleanVar(value=False)

    def reset():
        msg = Int32MultiArray()
        msg.data = [0, 0, 0]
        min_published.set(False)
        max_published.set(False)
        limit_publisher_.publish(msg)
        print("Reset limits")

    def publishmin():
        msg = Int32MultiArray()
        msg.data = [1, 0, 0]
        limit_publisher_.publish(msg)
        min_published.set(True)
        print("Published MIN limit")
    
    def publishmax():
        msg = Int32MultiArray()
        msg.data = [0, 1, 0]
        limit_publisher_.publish(msg)
        max_published.set(True)
        print("Published MAX limit")

    def publishoffset():
        if not min_published.get() or not max_published.get():
            messagebox.showwarning("Warning", "Limits must be define before declaring offset")
            return
        msg = Int32MultiArray()
        msg.data = [0, 0, 1]
        limit_publisher_.publish(msg)
        print("Published offset")

    def continuemain():
        if not min_published.get() or not max_published.get():
            messagebox.showerror("Error", "Limits must be define before continuing")
            return
        start_root.destroy()
        main()

    min_button = tk.Button(content_frame, text="Set MIN", command=publishmin, font=("Arial", 14), width=20, height=2, bg="blue", fg="white")
    max_button = tk.Button(content_frame, text="Set MAX", command=publishmax, font=("Arial", 14), width=20, height=2, bg="blue", fg="white")
    offset_button = tk.Button(content_frame, text="Set Offset", command=publishoffset, font=("Arial", 14), width=20, height=2, bg="blue", fg="white")
    reset_button = tk.Button(content_frame, text="Reset", command=reset, font=("Arial", 14), width=20, height=2, bg="red", fg="white")

    min_button.grid(row=0, column=1, padx=25, pady=20)
    max_button.grid(row=1, column=1, padx=25, pady=20)
    offset_button.grid(row=2, column=1, padx=25, pady=20)
    reset_button.grid(row=3, column=1, padx=25, pady=20)

    button_frame = ttk.Frame(start_root)
    button_frame.pack(side="bottom", pady=40)
    ttk.Button(button_frame, text="Continue", command=lambda: continuemain()).pack()
    
    start_root.mainloop()

if __name__ == '__main__':
    rclpy.init()
    startgui()
