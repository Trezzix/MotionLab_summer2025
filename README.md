# Motion Lab Summer Project 2025
- Summer project with "The Norwegian Motion Laboratory" at UiA campus Grimstad
- Control of industrial robot with suspended payload which is tracked with IMU sensors and cameras. Cameras are trained with YOLO object detection and read out location data from video stream
- Most communication runs over UDP, details in GitLab documentation below

<img src="COMAU_SpeedGoat/motion-lab.png" alt="drawing" height="80"/> 
<img src="Figures/MainPage_v1.jpg" alt="drawing" height="350"/> 

## General Information

The project is done by Thomas Lønne Stiansen and Adrian Mathias Lervik Ling, based on previous works with the repository's foundation from the works done by Johannes Arnesen Eidsvik (and Oliver Solberg?)
- https://github.com/JohannesAE/Comau_TCP_Control
- https://github.com/JohannesAE/UIA-MotionLab-DigitalTwin?tab=readme-ov-file

Public documentation of the Norwegian Motion Laboratory is found at \
https://uia-mekatronikk.gitlab.io/motion-lab/comau.html

### Known Bugs

- **KVM Switch:** Monitor may turn black/unresponsive, press CTRL+F12 then ESC
- **KVM Switch:** May become unresponsive when switching to an offline computer, press physical button to jump to another computer on the KVM switch
- **EM8000:** Ramp bug when going "To Settled", see red text in guide below.
- **PLC UDP Hangup:** No messages received on simulink model when running, restart PLC:
    - Can go online in "FB_ComauInterface" with TwinCAT in MotionLab in Visual Studio on Host PC #2 to ensure same signal is being read as in Simulink Data Inspector
    - Inside Host PC #2, search for "Remote Connection" in start menu
    - Use default IP 192.168.90.150 (may need password "1") for remote connection
    - START $\rightarrow$ Windows Security $\rightarrow$ Shut Down tab in bottom right corner $\rightarrow$ Restart
    - May need to restart TwinCAT in green "run mode" inside Visual Studio on Host PC #2 ("MotionLab" project)
- **Host PC HMI:** To start this: "C:GitLab/motion-lab-controller/hmi"
- **TwinCAT License Renewal:** May need renewal periodically
    - Solution Explorer $\rightarrow$ MotionLab $\rightarrow$ SYSTEM $\rightarrow$ License $\rightarrow$ License Activation $\rightarrow$ 7 Days Trial License...
- **SpeedGoat Unresponsive:** Reboot SpeedGoat from Simulink Real-Time Explorer
    - Simulink Real-Time Explorer can be opened from the following command in the MATLAB terminal: ```slrtExplorer```

### Comau Industrial Robot
#### Start

1) Turn physical switch on dark grey Comau cabinet $90^\circ$ CW
2) Comau TP5 Teach Pendant: 
    1) MENU $\rightarrow$ Prog $\rightarrow$ startC5GOpen $\rightarrow$ Program $\rightarrow$ Deactivate
    2) DRIVE (go from gray to green) *~ if not available, turn physical key to middle "TP"*
    3) (MENU $\rightarrow$ Prog $\rightarrow$) startC5GOpen $\rightarrow$ Program $\rightarrow$ Activate
    4) START
3) MATLAB terminal: slrtExplorer $\rightarrow$ connect to SpeedGoat
3) Changes to ```sg_IK.slx```? Run ```sg_IK_build.m```
4) Run ```sg_IK_startup.m```
5) MATLAB App: Set Stop Time = Inf
6) MATLAB App: Start

- *Ensure "Robot Operation Mode" is set to "REMOTE" under "Control Interface" in the Host PC (KVM switch #2) HMI*
- *Ensure Comau robot is in "Remote UDP Mode" in KVM switch #6*
- *Ensure Teach Pendant has key in correct orientation*
    - *Hand = Manual Mode ("Progr")*
    - *Logo $^{TP}$ = Program Mode ("Local")*
    - *Logo = ? ("Remote")*
- *Remember to use correct IP on laptop 192.168.1.15 to communicate with SpeedGoat*

#### Stop

1) MATLAB App: Enable Zero vel
2) MATLAB App: Stop, wait for "Run Time" to turn gray.
3) Comau TP5 Teach Pendant: DRIVE (go from green to gray)
3) [NOT REQUIRED, GOOD PRACTICE] startC5GOpen $\rightarrow$ Program $\rightarrow$ Deactivate
4) Turn off teach pendant: MENU $\rightarrow$ Home $\rightarrow$ Restart $\rightarrow$ Shutdown
5) After 30 [sec] (wait for update on pendant), turn physical switch on Comau cabinet $90^\circ$ CCW

#### Emergency

All steps are for the Comau TP5 Teach Pendant

1) Press down the big physical red button
2) To acknowledge and continue after emergency, turn button back and press the physical white restart button
3) The physical white restart button may also be pressed if "Power Link Error" occurs.

---

### Big Stewart E-Motion 8000 (EM8000)

#### Start
1) Turn on computer #4
2) Turn physical lever on big Rexroth cabinet $90^\circ$ CW
3) KVM Switch: Go to PC #4
    - May need to press "Acknowledge" on the error handler, as it has "FB Error Sercos hardware failure" when starting the PC before platform?
4) HyPCoS UIclient: (opened on startup) Open generator (either from main screen or Controls $\rightarrow$ Generator) and ensure zero values in this tab
5) HyPCoS UIclient: System Control $\rightarrow$ To Neutral $\rightarrow$ To Engaged
6) Signal Generator: Stop All before changing values
7) Signal Generator: Change values to desired (careful with yaw so it does not crash into machines nearby)
8) Signal Generator: Start All

<span style="color:red"> **BUG:** When enabling "To Settled", there is a known issue where it believes there is a ramp. To address this, wait for ramp down error and acknowledge (can take up to a minute approximately) before continuing. </span>

*A nice tip is to observe the "Actuators" GUI in the HyPCoS UIclient to ensure smooth operation and no jumps before engaging/starting signals.*

#### Stop
1) Signal Generator: Stop All
2) HyPCoS UIclient: System Control $\rightarrow$ To Neutral $\rightarrow$ To Settled
3) Double click "Halt" application on desktop
4) Turn physical lever on big Rexroth cabinet $90^\circ$ CCW

It should not be necessary to press "reset" if the error comes up to reset to normal operation, this can just be closed out.

---

### Little Stewart E-Motion 1500 (EM1500)

#### Start
1) Turn on computer #5
2) Turn physical switch on top of small Rexroth cabinet (in the corner) $90^\circ$ CW
3) KVM Switch: Go to PC #5
    - May need to press "Acknowledge" on the error handler, as it has "FB Error Sercos hardware failure" when starting the PC before platform?
4) HyPCoS UIclient: (opened on startup) Open generator (either from main screen or Controls $\rightarrow$ Generator) and ensure zero values in this tab
5) HyPCoS UIclient: System Control $\rightarrow$ To Neutral $\rightarrow$ To Engaged
6) Signal Generator: Stop All before changing values
7) Signal Generator: Change values to desired (careful with yaw so it does not crash into machines nearby)
8) Signal Generator: Start All

*A nice tip is to observe the "Actuators" GUI in the HyPCoS UIclient to ensure smooth operation and no jumps before engaging/starting signals.*

#### Stop
1) Signal Generator: Stop All 
2) HyPCoS UIclient: System Control $\rightarrow$ To Neutral $\rightarrow$ To Settled
3) Double click "Halt" application on desktop
4) Turn physical switch on top of small Rexroth cabinet (in the corner) $90^\circ$ CCW

It should not be necessary to press "reset" if the error comes up to reset to normal operation, this can just be closed out.

---

## File Structure
- ```COMAU_SpeedGoat``` = Project for controlling the industrial robot arm through Simulink model on SpeedGoat, continuing the work by Eidsvik
    - ```mr``` = Functions from "Modern Robotics" by Lynch & Park
    - ```Helper_functions``` = Custom functions for trajectory generation
    - ```sg_IK.slx``` SpeedGoat Inverse Kinematics Model (main)
    - ```sg_IK.mldatx``` SpeedGoat Inverse Kinematics Model, compiled to be uploaded
- ```machine_vision``` = Copy of project by Eidsvik for Claw & Ball tracking (template, only for inspiration of own)
- ```COMAU_Multibody``` = Multibody model of industrial robot, for simulation and frame visualization

## Prerequisites

- SpeedGoat Blockset: "9_4_0_1_R2021b_build_25363" to be installed on individual computer, accessed from SpeedGoat Customer Portal account or Daniel Hagen (responsible for the Motion Lab)
- Supported C++ Compiler: https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170#visual-studio-2015-2017-2019-and-2022
- MATLAB 2021b is required with the following addons:
    - Simulink Coder
    - MATLAB Coder
    - Signal Processing Toolbox
    - DSP System Toolbox
    - Simulink Real-Time Target Support Package
    - Simulink Real-Time
    - Simulink
    - Robotics System Toolbox (not sure)

## Components
- 1x 6-axis Robot Arm: COMAU NJ-110-3.0\
https://www.comau.com/en/our-offer/products-and-solutions/robot-team/nj-110-3-0/
- 2x Cameras: Luxonis OAK-D PRO\
https://oak-web.readthedocs.io/en/latest/components/nodes/imu/
https://docs.luxonis.com/software/depthai-components/nodes/imu
- <span style="color:red">X</span>x Motion Reference Unit: Kongsberg MRU

## To-Do List

This is a WIP section possibly to be removed later, serving as a checklist/notes for the summer project.

- Document system and previous progress (to a certain degree)
- Train YOLO on camera with new object
- Transformations between frames
    - Maybe change GAME_ROTATION_VECTOR as a known issue makes the camera frame pitched ~$20^\circ$
- Sensor fusion
- Enabled anti-pendulum by Ronny Landsverk
- Trajectory control with anti-pendulum algorithm active