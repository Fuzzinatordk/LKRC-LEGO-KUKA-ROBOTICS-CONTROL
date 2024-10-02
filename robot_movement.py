import subprocess,roboticstoolbox as rtb, numpy as np
from spatialmath import SE3
import ast, time
class robotMovement:
    # Class constructor
    def __init__(self):
        # File name for the generated python file
        self.fileName = "kukaCode.py"
        # Limits for the robot joints in degrees
        self.limitsDegrees = [[-170, 170], [-150,-10], [10, 155], [-350, 350], [-115, 115], [-350, 350]]
        # Limits for robot
        self.limitsCorrected = [[-170, 170], [-150,-30], [30, 155], [-350, 350], [-115, 115], [-350, 350]]
        self.limitsDegreesRobot = [-170,-150,155,-350,-115,-350]
        self.angular_error = [0,0,0,0,0,0]
        self.PTPList = []
        self.solutionList = []
        # Limits for the robot joints in radians
        self.limitsRadian = np.deg2rad(self.limitsCorrected)
        # Homing state for the robot
        self.homingState = False 
        self.firstRun = True
        self.checkRun = False
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
        #Homing
        #self.__writeFile(self.solutionList)
    def homing(self):
        self.homingState = False
        self.solutionList.clear()
        self.__writeFile(self.solutionList)    
    def teach(self):
        # Teaching the robot
        taught = self.kuka_robot.teach(self.kuka_robot.q,block=True,backend='pyplot',limits=[-300,300,-300,400,0,400])
        if taught:
            self.chosen_joint_config = [slider.val for slider in taught.sjoint]
            print(f'Chosen joint configuration: {self.chosen_joint_config} in degrees')
            return self.chosen_joint_config
    def linearPTP(self,pose=[0,0,0]):
        # Get the forward kinematics transformation matrix
        if len(pose) != 3:
            print('Please provide 3 pose values')
            return
        q = self.kuka_robot.fkine(self.kuka_robot.q)
        q_np = q.A  # Extract matrix
        q_np[0, 3] += pose[0]  # Modify x-value
        q_np[1, 3] += pose[1]  # Modify y-value
        q_np[2, 3] += pose[2]  # Modify z-value
        # Solve inverse kinematics for the modified transformation matrix
        q_ikine = self.kuka_robot.ikine_GN(q_np,joint_limits=True,slimit=100,ilimit=100)
        if not q_ikine.success:
            print('No solution found for the given pose')
            print(q_ikine.reason)
            return
        q_ikine = q_ikine.q
        self.kuka_robot.q = q_ikine
        q_ikine = np.rad2deg(q_ikine)
        q_ikine = q_ikine.tolist()
        print(f'End-effector pose: {q_ikine}')
        self.PTPList.append(q_ikine)
    def circularPTP(self,startPose=[0,0,0],endPose=[0,0,0],topPose=[0,0,0],steps=100):
        # Get the forward kinematics transformation matrix
        if len(startPose) != 3 or len(endPose) != 3 or len(topPose) != 3:
            print('Please provide 3 pose values')
            return
        start = np.array(startPose)
        end = np.array(endPose)
        top = np.array(topPose)
        # Calculate the Bezier curve
        t_values = np.linspace(0, 1, steps)
        #compute the points on the curve
        arc_points = [self.__bezierCurve(start, end, top, t) for t in t_values]
        for point in arc_points:
            q = self.kuka_robot.fkine(self.kuka_robot.q)
            q_np = q.A  # Extract matrix
            q_np[0, 3] += point[0]  # Modify x-value
            q_np[1, 3] += point[1] # Modify y-value
            q_np[2, 3] += point[2] # Modify z-value
            # Solve inverse kinematics for the modified transformation matrix
            q_ikine = self.kuka_robot.ikine_GN(q_np)
            if not q_ikine.success:
                print('No solution found for the given pose')
                print(q_ikine.reason)
                return
            q_ikine = q_ikine.q
            self.kuka_robot.q = q_ikine
            q_ikine = np.rad2deg(q_ikine)
            q_ikine = q_ikine.tolist()
            print(f'End-effector pose: {q_ikine}')
            self.PTPList.append(q_ikine)
        q = self.kuka_robot.fkine(self.kuka_robot.q)
        q_np = q.A
    def __bezierCurve(self, start, end, top, t):
        # Bezier curve calculation
        return (1-t)**2 * start + 2 * (1-t) * t * top + t**2 * end
    def linearRun(self):
        if len(self.PTPList) == 0:
            print('Please provide a pose first, using linearPTP()')
            return
        self.checkRun = True
        self.__writeFile(self.PTPList)
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
    def FK(self,angles,type='deg',plot=False,PTP=False):
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
        self.kuka_robot.q = angles
        angles = np.rad2deg(angles)
        angles = angles.tolist()
        self.solutionList.append(angles)
        # Writing the file
        self.__writeFile(self.solutionList)
        
          
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
        self.sol_lists = np.rad2deg(self.solution.q)
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
        self.solutionList.append(self.sol_lists)
        self.__writeFile(self.solutionList)
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
    def __writeFile(self, sols):
        self.corrected_angles_degrees = sols
        if self.firstRun and self.homingState == True:
            for sol in self.corrected_angles_degrees:
                for i in range(6):
                    if self.limitsDegreesRobot[i] > 0:
                        sol[i] += self.angular_error[i]
                    else:
                        sol[i] -= self.angular_error[i]
            print("Corrected angles: ",sols)
            self.firstRun = False
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
    f"correctJoints = {self.firstRun}\n"
    f"corrected_angles = {self.corrected_angles_degrees}\n"
    f"jointLimits = {self.limitsDegreesRobot}\n"
    f"list = {sols}\n"
    f"homingState = {self.homingState}\n"
    "homingMotorSpeed = 70\n"
    "motorSpeed = 100\n\n"
    "async def homingMotors():\n"
    "    await multitask(\n"
    "        Joint1.run_until_stalled(-homingMotorSpeed, then=Stop.COAST_SMART, duty_limit=30),\n"
    "        Joint2.run_until_stalled(-homingMotorSpeed, then=Stop.COAST_SMART, duty_limit=35),\n"
    "        Joint3.run_until_stalled(homingMotorSpeed, then=Stop.COAST_SMART, duty_limit=30),\n"
    "        Joint4.run_until_stalled(-homingMotorSpeed, then=Stop.COAST_SMART, duty_limit=15),\n"
    "        Joint5.run_until_stalled(-homingMotorSpeed, then=Stop.COAST_SMART, duty_limit=35),\n"
    "        Joint6.run_until_stalled(-homingMotorSpeed, then=Stop.COAST_SMART, duty_limit=15)\n"
    "    )\n\n"
    "def correctJoints():\n"
    "    for Joint,angle in zip([Joint1,Joint2,Joint3,Joint4,Joint5],corrected_angles):\n"
    "        Joint.reset_angle(angle)\n\n"
    "        print(Joint.angle())\n\n"
    "def setHomingLimits():\n"
    "    for Joint, limit in zip([Joint1,Joint2,Joint3,Joint4,Joint5,Joint6], jointLimits):\n"
    "        Joint.reset_angle(limit)\n\n"
    "def calcHomingDiff():\n"
    "    errorList = []\n"
    "    for Joint, limit in zip([Joint1,Joint2,Joint3,Joint4,Joint5,Joint6], jointLimits):\n"
    "        errorList.append(abs(Joint.angle() - limit))\n"
    "    return errorList\n\n"
    "def homing():\n"
    "    run_task(homingMotors())\n"
    "    wait(3000)\n"
    "    setHomingLimits()\n"
    "    print('error =',calcHomingDiff())\n"
    "    angles = [Joint1.angle(),Joint2.angle(),Joint3.angle(),Joint4.angle(),Joint5.angle(),Joint6.angle()]\n"
    "    print('Homing result =',angles)\n"
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
    "async def run_motors(list):\n"
    "    await multitask(\n"
    "        Joint1.run_angle(motorSpeed, calcDirection(list[0], Joint1), Stop.COAST_SMART),\n"
    "        Joint2.run_angle(motorSpeed, calcDirection(list[1], Joint2), Stop.COAST_SMART),\n"
    "        Joint3.run_angle(motorSpeed, calcDirection(list[2], Joint3), Stop.COAST_SMART),\n"
    "        Joint4.run_angle(motorSpeed, calcDirection(list[3], Joint4), Stop.COAST_SMART),\n"
    "        Joint5.run_angle(motorSpeed, calcDirection(list[4], Joint5), Stop.COAST_SMART),\n"
    "        Joint6.run_angle(motorSpeed, calcDirection(list[5], Joint6), Stop.COAST_SMART)\n"
    "    )\n\n"
    
    "async def driveMotors(direction_list):\n"
    "    await multitask(run_motors(direction_list))\n\n"
    
    "def main():\n"
    "    if correctJoints == False:\n"
    "       correctJoints()\n"
    "    if homingState == False:\n"
    "       homing()\n"
    "       return\n"
    "    else:\n"
    "       for direction_list in list:\n"
    "           run_task(driveMotors(direction_list))\n"
    "           wait(1000)\n\n"
    "       angles = [Joint1.angle(),Joint2.angle(),Joint3.angle(),Joint4.angle(),Joint5.angle(),Joint6.angle()]\n"
    "       print('Angle result =',angles)\n"
    
    "main()\n"
    "hub.speaker.beep(1000,500)\n"
        )
        # Writing the file
        print("Writing file...")
        self.__createFile(content)
    def __terminalCmd(self, command):
        # Running the terminal command
        result = subprocess.run(command, shell=True, capture_output=True, text=True, encoding='utf-8')
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
                for line in result.stdout.splitlines():
                       if "error =" in line and self.homingState == False:
                        error_str = line.split("error =")[-1].strip()
                        self.angular_error = ast.literal_eval(error_str)
                if self.homingState == False:
                    self.homingState = True
                if self.checkRun == True:
                    self.PTPList.clear()
                    self.checkRun = False
                    
                return

            
