import sys, subprocess,roboticstoolbox as rtb, numpy as np
from spatialmath import SE3
import keyboard, time
class robotMovement:
    # Class constructor
    def __init__(self):
        # File name for the generated python file
        self.fileName = "kukaCode.py"
        # Limits for the robot joints in degrees
        self.limitsDegrees = [[-170, 170], [-150,-25], [40, 155], [-350, 335], [-115, 100], [-350, 335]]
        # Limits for robot
        self.limitsDegreesRobot = [-170,-150,155,-350,-115,-350]
        # Limits for the robot joints in radians
        self.limitsRadian = np.deg2rad(self.limitsDegrees)
        # Homing state for the robot
        self.homingState = False 
        # DH parameters for the robot   
        self.__DHParams()
    # Private method to set the DH parameters for the robot
    def __DHParams(self):
        # DH parameters for the robot
        self.dh_params = [
            [-np.pi/2, 16, 132, 0],                           
            [0,112, 0, 0],            
            [-np.pi/2, 40, 0, -np.pi/2],                    
            [np.pi/2, 0, 160,0],         
            [-np.pi/2, 0, 0, 0],                    
            [0, 0, 44,np.pi]                 
        ]
        # Creating the robot
        self.links = []
        # Creating the links for the robot
        for param in self.dh_params:
            # Creating the link
            self.link = rtb.RevoluteDH(d=param[2], a=param[1], alpha=param[0], offset=param[3])
            # Appending the link to the links list
            self.links.append(self.link)
        self.kuka_robot = rtb.DHRobot(self.links, name='KUKA 6-DOF')
        # Setting the limits for the robot
        limitArray = np.ndarray(shape=(2,6),dtype=float, order='F', buffer=self.limitsRadian)
        self.q0 = np.zeros(6)
        for i in range(6):
            if i == 2:
                self.q0[i] = (self.limitsRadian[i][1])
                continue
            self.q0[i] = (self.limitsRadian[i][0])
        # Setting the initial joint angles for the robot
        self.q0home = np.ndarray(shape=(1,6),dtype=float, order='F', buffer=self.q0)
        self.kuka_robot.q = self.q0home
        self.kuka_robot.qlim = limitArray
    def FK_solution(self,angles,type):
        # Checking if the number of angles provided is correct
        if len(angles) != 6:
            print('Please provide 6 joint angles')
            return
        # Checking if the angles are in degrees
        self.limits = self.limitsDegrees
        if type == 'radian' or type == 'rad':
            angles = np.rad2deg(angles)
            angles = angles.tolist()
        # Checking if the angles are within the limits
        for i in range(6):
            if angles[i] < self.limits[i][0] or angles[i] > self.limits[i][1]:
                print(f'Joint {i+1} is out of limits')
                return
        # Calculating the forward kinematics
        self.solution = self.kuka_robot.fkine(angles)
        # Displaying the end-effector pose
        print(f'End-effector pose: {self.solution}')
        return self.solution
    def DH(self):
        # Displaying the DH parameters for the robot
        print(self.kuka_robot)
    def getJacob(self):
        # Getting the Jacobian matrix for the robot
        J = self.kuka_robot.jacob0(self.q0)
        print(J)
    def FK(self,angles,type,plot=False,PTP=False):
        # Forward kinematics for the robot
        # Checking if the number of angles provided is correct
        if len(angles) != 6:
            print('Please provide 6 joint angles')
            return
        # Checking if the angles are in degrees
        self.limits = self.limitsRadian
        if type == 'deg' or type == 'degrees':
            angles = np.deg2rad(angles)
            angles = angles.tolist()
        # Checking if the angles are within the limits
        for i in range(6):
            if angles[i] < self.limits[i][0] or angles[i] > self.limits[i][1]:
                print(f'Joint {i+1} is out of limits')
                return
        # Calculating the forward kinematics
        self.solution = self.kuka_robot.fkine(angles)
        # Displaying the end-effector pose
        print(f'End-effector pose: {self.solution}')
        if plot:
            # Plotting the robot
            self.posePlot(angles)   
        if PTP:
            # PTP motion
            self.PTPplot(angles)
        #Stating new q0 for the robot to start from 
        self.kuka_robot.q = np.ndarray(shape=(1,6),dtype=float, order='F', buffer=angles)
        angles = np.rad2deg(angles)
        angles = angles.tolist()
        # Writing the file
        self.writeFile(angles)
        
          
    def IK(self,pose,type,plot=False,PTP=False):
        # Inverse kinematics for the robot
        if len(pose) != 6:
            print('Please provide 6 pose values')
            return
        # Checking if the angles are in degrees
        if type == 'degrees' or type == 'deg':
            pose[3] = np.deg2rad(pose[3])
            pose[4] = np.deg2rad(pose[4])
            pose[5] = np.deg2rad(pose[5])
        # Calculating the transformation matrix
        T = SE3.Trans(pose[0], pose[1], pose[2]) * SE3.RPY(pose[3], pose[4], pose[5])
        # Calculating the inverse kinematics
        self.solution = self.kuka_robot.ikine_GN(T,joint_limits=True,slimit=100,ilimit=100)
        if self.solution.success:
           print(f'Residual error for the solution found: {self.solution.residual}')
           print(f'Solution found: {self.solution.q}')
        else:                       
            print(self.solution.reason)
            print('No solutions found for the given pose.')
            return
        # Displaying the joint angles
        self.sol_lists = [0] * len(self.solution.q)
        self.sol_lists = self.solution.q * 180 / np.pi
        self.sol_lists = self.sol_lists.tolist()
        # Writing the file
        if plot:
            # Plotting the robot
            self.posePlot(self.solution.q)
        if PTP:
            # PTP motion
            self.PTPplot(self.solution.q)
        # Stating new q0 for the robot to start from
        self.kuka_robot.q = np.ndarray(shape=(1,6),dtype=float, order='F', buffer=self.solution.q)
        self.writeFile(self.sol_lists)
    def randomPose(self, plot=False,PTP=False):
        # Generating random joint angles for the robot
        randomQ = np.random.uniform(self.limitsRadian[:,0],self.limitsRadian[:,1])
        self.FK(randomQ,'rad',plot,PTP)
    def limitsDeg(self):
        # Displaying the limits for the robot in degrees
        print(self.limitsDegrees)
    def limitsRad(self):
        # Displaying the limits for the robot in radians
        print(self.limitsRadian)
    def PTPplot(self,angles):
        # PTP motion for the robot
        traj = rtb.jtraj(self.kuka_robot.q, angles, 100)
        rtb.xplot(traj.q,block=True)
    def posePlot(self,pose):
        # Plotting the robot
        q0old = self.kuka_robot.q.copy()
        traj = rtb.jtraj(self.kuka_robot.q,pose, 100)
        self.kuka_robot.plot(traj.q,block=True,backend='pyplot', eeframe=True,dt=0.05,limits=[-300,300,-300,400,0,400])
        self.kuka_robot.q = q0old
        
    def writeFile(self, sols):
        # Writing the python file for the robot
        content = (
    "from pybricks.hubs import InventorHub\n"
    "from pybricks.pupdevices import Motor\n"
    "from pybricks.parameters import Port, Direction, Stop\n"
    "from pybricks.tools import wait, StopWatch, multitask, run_task\n\n"
    
    "hub = InventorHub()\n"
    "Joint1 = Motor(Port.A, reset_angle=False, gears=[1,37])\n"
    "Joint2 = Motor(Port.E, Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[1,67])\n"
    "Joint3 = Motor(Port.C, reset_angle=False, gears=[1,52])\n"
    "Joint4 = Motor(Port.B, Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[1,21])\n"
    "Joint5 = Motor(Port.D, reset_angle=False, gears=[1,22])\n"
    "Joint6 = Motor(Port.F, Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[1,12])\n"
    f"jointLimits = {self.limitsDegreesRobot}\n"
    f"direction_list = {sols}\n"
    f"homingState = {self.homingState}\n"
    "motorSpeed = 500\n"
    "dcDefault = 50\n"
    "def duty_cycle_adjustment():\n"
    "   Joint1.dc(dcDefault)\n"
    "   Joint2.dc(dcDefault)\n"
    "   Joint3.dc(dcDefault)\n"
    "   Joint4.dc(dcDefault)\n"
    "   Joint5.dc(dcDefault)\n"
    "   Joint6.dc(dcDefault)\n\n"
    "def motorErrorCompen():\n"
    "    for i,(Joint, limit) in enumerate(zip([Joint1,Joint2,Joint3,Joint4], jointLimits)):\n"
    "        if Joint.angle() != limit:\n"  
    "            error = abs(calcDirection(limit, Joint))\n"
    "            if direction_list[i] < 0:\n"
    "                direction_list[i] -= error\n"
    "            else:\n"
    "                direction_list[i] += error\n\n"
    "async def homingMotors():\n"
    "    await multitask(\n"
    "        #Joint1.run_until_stalled(-motorSpeed, then=Stop.COAST_SMART, duty_limit=40),\n"
    "        Joint2.run_until_stalled(-motorSpeed, then=Stop.COAST_SMART, duty_limit=35),\n"
    "        Joint3.run_until_stalled(motorSpeed, then=Stop.COAST_SMART, duty_limit=30),\n"
    "        Joint4.run_until_stalled(-motorSpeed, then=Stop.COAST_SMART, duty_limit=15),\n"
    "        Joint5.run_until_stalled(-motorSpeed, then=Stop.COAST_SMART, duty_limit=35)\n"
    "    )\n\n"
    
    "def setHomingLimits():\n"
    "    for Joint, limit in zip([Joint1,Joint2,Joint3,Joint4,Joint5], jointLimits):\n"
    "        Joint.reset_angle(limit)\n\n"  
    "def homing():\n"
    "    run_task(homingMotors())\n"
    "    wait(3000)\n"
    "    duty_cycle_adjustment()\n"
    "    motorErrorCompen()\n"
    "    setHomingLimits()\n"
    "    return True\n\n"
    
    "async def check_motor():\n"
    "    return all(Joint.done() for Joint in [Joint1,Joint2,Joint3,Joint4,Joint5,Joint6])\n\n"
    
    "def calcDirection(dir, Joint):\n"
    "    angleCalc = Joint.angle()\n"
    "    if dir > 0:\n"
    "        if angleCalc < 0:\n"
    "            angleCalc = abs(angleCalc) + dir\n"
    "        else:\n"
    "            angleCalc = dir - angleCalc\n"
    "    elif dir < 0:\n"
    "        if angleCalc < 0:\n"
    "            angleCalc = abs(angleCalc) + dir\n"
    "        else:\n"
    "            angleCalc = dir - angleCalc\n"
    "    else:\n"
    "        if angleCalc < 0:\n"
    "           angleCalc = abs(angleCalc) \n"
    "        else:\n"
    "           angleCalc = -1 * angleCalc\n" 
    "    return angleCalc\n\n"
    "async def stallMotors():\n"
    "    if Joint1.stalled():\n"
    "        Joint1.stop()\n"
    "        print('Joint1 stalled at',Joint1.angle())\n"
    "    if Joint2.stalled():\n"
    "        Joint2.stop()\n"
    "        print('Joint2 stalled at',Joint2.angle())\n"
    "    if Joint3.stalled():\n"
    "        Joint3.stop()\n"
    "        print('Joint3 stalled at',Joint3.angle())\n"
    "    if Joint4.stalled():\n"
    "        Joint4.stop()\n"
    "        print('Joint4 stalled at',Joint4.angle())\n"
    "    if Joint5.stalled():\n"
    "        Joint5.stop()\n"
    "        print('Joint5 stalled at',Joint5.angle())\n"
    "    if Joint6.stalled():\n"
    "        Joint6.stop()\n\n"
    "        print('Joint6 stalled at',Joint6.angle())\n\n"
        
    "async def run_motors(list):\n"
    "    await multitask(\n"
    "        #Joint1.run_angle(motorSpeed, calcDirection(list[0], Joint1), Stop.COAST_SMART),\n"
    "        Joint2.run_angle(motorSpeed, calcDirection(list[1], Joint2), Stop.COAST_SMART),\n"
    "        Joint3.run_angle(motorSpeed, calcDirection(list[2], Joint3), Stop.COAST_SMART),\n"
    "        Joint4.run_angle(motorSpeed, calcDirection(list[3], Joint4), Stop.COAST_SMART),\n"
    "        Joint5.run_angle(motorSpeed, calcDirection(list[4], Joint5), Stop.COAST_SMART),\n"
    "        Joint6.run_angle(motorSpeed, calcDirection(list[5], Joint6), Stop.COAST_SMART)\n"
    "    )\n\n"
    
    "async def print_angles(watch):\n"
    "    print(f\"{'Joint':>10} {'J1':>10} {'J2':>10} {'J3':>10} {'J4':>10} {'J5':>10} {'J6':>10}\")\n"
    "    print(f\"{'Target':>10} {direction_list[0]:>10.2f} {direction_list[1]:>10.2f} {direction_list[2]:>10.2f} {direction_list[3]:>10.2f} {direction_list[4]:>10.2f} {direction_list[5]:>10.2f}\")\n"
    "    while True:\n"
    "        angles = [\n"
    "            Joint1.angle(),\n"
    "            Joint2.angle(),\n"
    "            Joint3.angle(),\n"
    "            Joint4.angle(),\n"
    "            Joint5.angle(),\n"
    "            Joint6.angle()\n"
    "        ]\n"
    "        if await check_motor():\n"
    "            print('Elapsed time:',watch.time()/1000,'s')\n"
    "            print(f\"{'Error':>10} {abs(direction_list[0]) - abs(angles[0]):>10.2f} {abs(direction_list[1]) - abs(angles[1]):>10.2f} {abs(direction_list[2]) - abs(angles[2]):>10.2f} {abs(direction_list[3]) - abs(angles[3]):>10.2f} {abs(direction_list[4]) - abs(angles[4]):>10.2f} {abs(direction_list[5]) - abs(angles[5]):>10.2f}\")\n"
    "            print(f\"{'Angle result':>10} {angles[0]:>10.2f} {angles[1]:>10.2f} {angles[2]:>10.2f} {angles[3]:>10.2f} {angles[4]:>10.2f} {angles[5]:>10.2f}\")\n"
    "            print('Jobs done')\n"
    "            break\n\n"
   
    "        await wait(1000)\n\n"
    "        print(f\"{'Current angle':>10} {angles[0]:>10.2f} {angles[1]:>10.2f} {angles[2]:>10.2f} {angles[3]:>10.2f} {angles[4]:>10.2f} {angles[5]:>10.2f}\")\n"
    
    "async def driveMotors(watch):\n"
    "    await multitask(run_motors(direction_list),print_angles(watch))\n\n"
    
    "def main():\n"
    "    if homingState == False:\n"
    "       homing()\n"
    "    watch = StopWatch()\n"
    "    run_task(driveMotors(watch))\n"
    "    watch.reset()\n\n"
    
    "main()\n"
    "hub.speaker.beep(1000,500)\n"
        )
        # Writing the file
        print("Writing file...")
        self.__createFile(content)
    def __terminalCmd(self, command):
        # Running the terminal command
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        print(result.stdout)
        return result
    def __createFile(self,content):
        # Creating the python file
        with open(self.fileName, 'w') as file:
            file.write(content)
        self.__runFile()   
    def __runFile(self):
        # Running the python file
        #if self.homingState == False:
            #print("Press 'q' to run the program, make sure bluetooth on your device is on and the robot is turned on")
            #keyboard.wait('q')
        print("Running program...")
        command = f'pipx run pybricksdev run ble {self.fileName}'
        while(True):
            result = self.__terminalCmd(command)
            time.sleep(0.5)
            if(result.stdout != 0):
                break
        if self.homingState == False:
            self.homingState = True
    
        
