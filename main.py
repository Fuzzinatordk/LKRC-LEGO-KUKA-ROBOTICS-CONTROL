import tkinter as tk
from robot_movement import robotMovement
from GUI import RobotGUI

def main():
    root = tk.Tk()
    root.title("Robot GUI")
    root.geometry("400x400")
    robot = robotMovement()
    gui = RobotGUI(root, robot)
    root.mainloop()

if __name__ == "__main__":
    main()