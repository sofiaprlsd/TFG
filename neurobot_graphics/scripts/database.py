#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import csv
import os

HOME_DIR = os.path.expanduser("~")
DATABASE_DIR = os.path.join(HOME_DIR, "database")
os.makedirs(DATABASE_DIR, exist_ok=True)

def exit():
    root.quit()

def clear():
    name_var.set("")
    surname_var.set("")
    id_var.set("")
    freq_var.set("")
    ampl_var.set("")
    disturb_var.set("")
    duration_var.set("")
    period_var.set("")
    level_var.set("")
    progress_var.set("")
    notes_text.delete("1.0", "end")

def savedata():
    name = name_var.get().strip()
    surname = surname_var.get().strip()
    id = id_var.get().strip()
    freq = freq_var.get().strip()
    ampl = ampl_var.get().strip()
    disturb = disturb_var.get().strip()
    duration = duration_var.get().strip()
    period = period_var.get().strip()
    level = level_var.get().strip()
    progress = progress_var.get().strip()
    notes = notes_text.get("1.0", "end").strip()

    if not (id.isdigit()):
        messagebox.showwarning("Invalid input", "ID must be a number")
        return

    ext = ".csv"
    ID_DIR = os.path.join(DATABASE_DIR, id)
    os.makedirs(ID_DIR, exist_ok=True)
    file_name = f"{id}{ext}"
    file_path = os.path.join(ID_DIR, file_name)

    patient_data = {
        "name": name,
        "surname": surname,
        "ID": id,
        "frequency": freq,
        "amplitude": ampl,
        "disturbance": disturb,
        "duration": duration,
        "period": period,
        "level": level,
        "progress": progress,
        "notes": notes
    }

    file_exists = os.path.isfile(file_path)
    with open(file_path, mode="a", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=patient_data.keys())
        if not file_exists:
            writer.writeheader()
        writer.writerow(patient_data)

    messagebox.showinfo("Saved", f"Data saved to {file_name}")
    clear()

# ----------------------- Graphic Interface ----------------------- #

root = tk.Tk()
root.title("database")
root.geometry("600x500")
root.resizable(False, False)

style = ttk.Style()
style.configure("TLabel", font=("Segoe UI", 10))
style.configure("TEntry", font=("Segoe UI", 10))
style.configure("TButton", font=("Segoe UI", 10, "bold"))
style.configure("TFrame", padding="10 10 10 10")

main_frame = ttk.Frame(root)
main_frame.pack(fill="both", expand=True)

button_frame = ttk.Frame(main_frame)
button_frame.grid(column=1, row=12, pady=10, sticky=tk.W)

name_var = tk.StringVar()
surname_var = tk.StringVar()
id_var = tk.StringVar()
freq_var = tk.StringVar()
ampl_var = tk.StringVar()
disturb_var = tk.StringVar()
duration_var = tk.StringVar()
period_var = tk.StringVar()
level_var = tk.StringVar()
progress_var = tk.StringVar()

ttk.Label(main_frame, text="Patient", font=("Segoe UI", 12, "bold")).grid(column=0, row=0, columnspan=2, pady=10)

ttk.Label(main_frame, text="Name:").grid(column=0, row=1, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=name_var).grid(column=1, row=1, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="Surname:").grid(column=0, row=2, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=surname_var).grid(column=1, row=2, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="ID:").grid(column=0, row=3, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=id_var).grid(column=1, row=3, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="Frequency:").grid(column=0, row=4, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=freq_var).grid(column=1, row=4, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="Amplitude:").grid(column=0, row=5, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=ampl_var).grid(column=1, row=5, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="Disturbance:").grid(column=0, row=6, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=disturb_var).grid(column=1, row=6, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="Disturbance duration:").grid(column=0, row=7, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=duration_var).grid(column=1, row=7, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="Disturbance period:").grid(column=0, row=8, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=period_var).grid(column=1, row=8, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="Level:").grid(column=0, row=9, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=level_var).grid(column=1, row=9, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="Progress (%):").grid(column=0, row=10, sticky=tk.E, pady=5)
ttk.Entry(main_frame, width=30, textvariable=progress_var).grid(column=1, row=10, sticky=tk.W, pady=5)

ttk.Label(main_frame, text="Doctor notes:").grid(column=0, row=11, sticky=tk.NE, pady=5)
notes_text = tk.Text(main_frame, width=30, height=5, font=("Segoe UI", 10))
notes_text.grid(column=1, row=11, sticky=tk.W, pady=5)

save_btn = tk.Button(button_frame, text="Save", command=savedata, bg="green", fg="white", font=("Segoe UI", 10), width=10)
save_btn.pack(side="left", padx=(0, 10))

exit_btn = tk.Button(button_frame, text="Exit", command=exit, bg="red", fg="white", font=("Segoe UI", 10), width=10)
exit_btn.pack(side="left")

main_frame.columnconfigure(0, weight=1)
main_frame.columnconfigure(1, weight=2)

root.mainloop()
