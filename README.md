# Delta
Manufactured and programmed a Delta robot from scratch. This robot is the result of a group project over the course of one semester. The objective was to design, manufacture and program a 2DOF delta robot, so that in conjunction with an industrial camera the robot could execute a 'pick and place' routine on a bottle cap. The robot was mounted to a static base above two conveyors; the camera was mounted to the side of one of the conveyors with its field of vision right above the conveyor carrying the bottle cap. The robot was designed to be operational via a simple HMI screen using Siemens’ own software. The industrial application begins with the conveyor advancing and stopping once the bottle cap is within the cameras range; once stopped the camera takes a picture reveling the bottle cap’s coordinates. The conveyor advances once again and stops when it is aligned with the robot’s coordinate plane of movement. Having the cap’s position, the robot now executes the pick and place routine moving the bottle cap to the other conveyor. The robot was programmed with a Siemens 1200 PLC and an ESP32 Pico-Kit V4. It uses two stepper motors, a switched-mode power source and a Cognex 7802 camera along with its In-sight integrated software.

My main roles included the use of a CNC milling machine, a conventional milling machine, and a conventional lathe machine to create the mechanical components of the robot. The source material used was aluminum. My second role was to program the camera within the In-Sight software to detect bottle caps, calculate their coordinates and send them to the PLC via an ethernet protocol. My third role was to design a simple HMI interface using Siemens semantic; the HMI would allow the user to trigger the camera by software, insert desired absolute positions for the delta, execute the pick and place routine, and have real time visualizing bars to indicate the speed, angles and position of the robot’s arms. My fourth role in the team was to translate the team’s existing PLC code to Arduino’s framework in order to use a microcontroller (ESP32) that has an integrated Wi-Fi module. The purpose of using this microcontroller was to create a webpage, and connect the microcontroller to the internet so anyone in the world could potentially move the Delta robot. For this last functionality to be possible an IP port forwarding software was used called ‘Ngrok’.

In this repository one can find the code implemented; picture and video footage of the manufacturing process, the webpage, the HMI, The code for the Cognex camera and the robot's functionality.

# Linked videos 
[Delta functionality](https://youtu.be/MyCAaGi_ooI)

[Milling CNC Working](https://youtu.be/NQq0tLHyiK0)

# Human Machine Interface
![HMI_Img](HMI_Img.png)




