import tkinter as tk
from robot_movement import robotMovement
from tkinter import scrolledtext

class RobotGUI:
    def __init__(self,root,robot):
        self.root = root
        self.robot = robot
        buttonFK = tk.Button(root, text="Forward Kinematics", command=self.FK)
        buttonFK.pack()
        buttonIK = tk.Button(root, text="Inverse Kinematics", command=self.IK)
        buttonIK.pack()
        buttonRandomPose = tk.Button(root, text="Random Pose", command=self.RandomPose)
        buttonRandomPose.pack()
        self.terminal = scrolledtext.ScrolledText(root, wrap=tk.WORD, width=40, height=10)
        self.terminal.pack()
    def FK(self):
        self.terminal.insert(tk.END, self.robot.FK() + "\n")
    def IK(self):
        self.terminal.insert(tk.END, self.robot.IK() + "\n")
    def RandomPose(self):
        self.terminal.insert(tk.END, self.robot.RandomPose() + "\n")
        
        