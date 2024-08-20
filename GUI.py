import tkinter as tk
from tkinter import ttk
import numpy as np
import threading
import signal
import sys
from robot_movement import robotMovement  # Assuming your class is in a file named robot_movement.py

class RobotMovementGUI:
    def __init__(self, root):
        self.root = root
        self.robot = robotMovement()
        self._stop_event = threading.Event()  # Event to signal threads to stop

        # Setting up the main frame
        self.frame = ttk.Frame(root, padding="10")
        self.frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Forward Kinematics (FK) Section
        ttk.Label(self.frame, text="Forward Kinematics (FK)").grid(row=0, column=0, columnspan=2, sticky=tk.W)
        self.fk_angles_entry = ttk.Entry(self.frame)
        self.fk_angles_entry.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E))
        ttk.Label(self.frame, text="Angles (comma-separated)").grid(row=2, column=0, columnspan=2, sticky=tk.W)
        
        self.fk_units = tk.StringVar(value="degrees")
        self.fk_units_menu = ttk.OptionMenu(self.frame, self.fk_units, "degrees", "radians")
        self.fk_units_menu.grid(row=3, column=0, sticky=tk.W)
        ttk.Label(self.frame, text="Units").grid(row=3, column=1, sticky=tk.W)
        
        self.fk_plot_var = tk.BooleanVar()
        self.fk_plot_check = ttk.Checkbutton(self.frame, text="Plot", variable=self.fk_plot_var)
        self.fk_plot_check.grid(row=4, column=0, sticky=tk.W)
        
        self.fk_ptp_var = tk.BooleanVar()
        self.fk_ptp_check = ttk.Checkbutton(self.frame, text="PTP", variable=self.fk_ptp_var)
        self.fk_ptp_check.grid(row=4, column=1, sticky=tk.W)
        
        self.fk_button = ttk.Button(self.frame, text="Calculate FK", command=lambda: self.thread_function(self.calculate_fk))
        self.fk_button.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E))

        # Inverse Kinematics (IK) Section
        ttk.Label(self.frame, text="Inverse Kinematics (IK)").grid(row=6, column=0, columnspan=2, sticky=tk.W)
        self.ik_pose_entry = ttk.Entry(self.frame)
        self.ik_pose_entry.grid(row=7, column=0, columnspan=2, sticky=(tk.W, tk.E))
        ttk.Label(self.frame, text="Pose (comma-separated)").grid(row=8, column=0, columnspan=2, sticky=tk.W)
        
        self.ik_units = tk.StringVar(value="degrees")
        self.ik_units_menu = ttk.OptionMenu(self.frame, self.ik_units, "degrees", "radians")
        self.ik_units_menu.grid(row=9, column=0, sticky=tk.W)
        ttk.Label(self.frame, text="Units").grid(row=9, column=1, sticky=tk.W)
        
        self.ik_plot_var = tk.BooleanVar()
        self.ik_plot_check = ttk.Checkbutton(self.frame, text="Plot", variable=self.ik_plot_var)
        self.ik_plot_check.grid(row=10, column=0, sticky=tk.W)
        
        self.ik_ptp_var = tk.BooleanVar()
        self.ik_ptp_check = ttk.Checkbutton(self.frame, text="PTP", variable=self.ik_ptp_var)
        self.ik_ptp_check.grid(row=10, column=1, sticky=tk.W)
        
        self.ik_button = ttk.Button(self.frame, text="Calculate IK", command=lambda: self.thread_function(self.calculate_ik))
        self.ik_button.grid(row=11, column=0, columnspan=2, sticky=(tk.W, tk.E))

        # Random Pose Section
        ttk.Label(self.frame, text="Random Pose").grid(row=12, column=0, columnspan=2, sticky=tk.W)
        
        self.random_plot_var = tk.BooleanVar()
        self.random_plot_check = ttk.Checkbutton(self.frame, text="Plot", variable=self.random_plot_var)
        self.random_plot_check.grid(row=13, column=0, sticky=tk.W)
        
        self.random_ptp_var = tk.BooleanVar()
        self.random_ptp_check = ttk.Checkbutton(self.frame, text="PTP", variable=self.random_ptp_var)
        self.random_ptp_check.grid(row=13, column=1, sticky=tk.W)
        
        self.random_button = ttk.Button(self.frame, text="Generate Random Pose", command=lambda: self.thread_function(self.run_random_pose))
        self.random_button.grid(row=14, column=0, columnspan=2, sticky=(tk.W, tk.E))
        
        # Exit Button
        self.exit_button = ttk.Button(self.frame, text="Exit", command=self.exit_program)
        self.exit_button.grid(row=15, column=0, columnspan=2, sticky=(tk.W, tk.E))

        # Handle window close event
        root.protocol("WM_DELETE_WINDOW", self.exit_program)

    def get_selected_units(self, is_fk):
        return self.fk_units.get() if is_fk else self.ik_units.get()
    
    def convert_angles(self, angles, units):
        if units == "radians":
            return np.deg2rad(angles)
        return angles
    
    def calculate_fk(self):
        if self._stop_event.is_set():
            return
        angles_str = self.fk_angles_entry.get()
        try:
            angles = [float(a) for a in angles_str.split(',')]
            if len(angles) != 6:
                raise ValueError("Please provide exactly 6 angles.")
            units = self.get_selected_units(is_fk=True)
            angles = self.convert_angles(angles, units)
            plot = self.fk_plot_var.get()
            ptp = self.fk_ptp_var.get()
            self.robot.FK(angles, "rad" if units == "radians" else "deg", plot=plot, PTP=ptp)
        except ValueError as e:
            self.show_error(str(e))

    def calculate_ik(self):
        if self._stop_event.is_set():
            return
        pose_str = self.ik_pose_entry.get()
        try:
            pose = [float(p) for p in pose_str.split(',')]
            print(pose)
            if len(pose) != 6:
                raise ValueError("Please provide exactly 6 pose values.")
            units = self.get_selected_units(is_fk=False)
            pose_angles = self.convert_angles(pose[3:], units)
            pose = pose[:3] + pose_angles
            plot = self.ik_plot_var.get()
            ptp = self.ik_ptp_var.get()
            self.robot.IK(pose, "rad" if units == "radians" else "deg", plot=plot, PTP=ptp)
        except ValueError as e:
            self.show_error(str(e))
    
    def run_random_pose(self):
        if self._stop_event.is_set():
            return
        plot = self.random_plot_var.get()
        ptp = self.random_ptp_var.get()
        self.robot.randomPose(plot=plot, PTP=ptp)
    
    def thread_function(self, func):
        # Set the stop event to False before starting a new thread
        self._stop_event.clear()
        threading.Thread(target=func).start()
    
    def exit_program(self):
        # Signal the threads to stop
        self._stop_event.set()
        
        # Ensure all threads have finished
        self.root.quit()
        self.root.destroy()
        
        # Optionally, add a short delay to allow threads to finish
        # time.sleep(1)
        # os._exit(0)  # Use this to forcefully exit if necessary

    def show_error(self, message):
        error_window = tk.Toplevel(self.frame)
        error_window.title("Error")
        ttk.Label(error_window, text=message).pack(padx=10, pady=10)
        ttk.Button(error_window, text="OK", command=error_window.destroy).pack(padx=10, pady=10)

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Robot Movement GUI")
    app = RobotMovementGUI(root)
    root.mainloop()
