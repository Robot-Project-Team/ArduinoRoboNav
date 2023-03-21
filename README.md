A DEMO for the cockpit of the virtual robot:
![IMG_7050](https://user-images.githubusercontent.com/60235671/226649807-a5863d6a-033d-4e11-8e3f-b8737a61b178.jpg)
A link to the video of controlling the robot:
https://drive.google.com/file/d/1fNZmQ_goI6pZimLXsUi_oXv1Iwmf9GLh/view?usp=share_link

# roboNav
Users are able to control the robot with Arduino in the Virtual robot navigation environment made in Unreal Engine 4.26.2. The robot is currently represented by a trapezoidal prism, with rigid body physics enabled.

Below explains:
1. Role of each folders
2. Get Started in Unreal engine

 - Screenshot of robot in a 2.5D environment:
![image](https://user-images.githubusercontent.com/60235671/126192124-3532222e-b3f1-4ab8-9f20-414f865b5bee.png)

### Role of each folders:

- RoboNav_new_chair -- UE4 project for Chair Chair design. See "Get Started" section below before running it.
- IMU_UE4_chair -- Arduino code for chair chair design. Upload it to Arduino before running RoboNav_new_chair.

- RoboNav_new_wobble -- UE4 project for wobble board design.
- IMU_UE4_wobble -- Arduino code for wobble board design. Upload it to Arduino before running RoboNav_new_wobble.
#### Code for hardware debugging:
- Accel_test2 -- Arduino code. If needed, upload it to Arduino to check the hardware connection of IMU pins with Arduino pins. It will also show the accelerometer readings.
- FSR_cpp -- Arduino code. If needed, upload it to Arduino to check the force measurement of the two FSR sensors.

### Get Started:
1. Upload code in Arduino IDE
2. In Unreal Engine 4, open Blueprints of TopDown Character. Change the Serial port number according to the COM Port used by Arduino. 
3. Click "Play" button and the game will start.
![image](https://user-images.githubusercontent.com/60235671/126820836-14178afe-d43d-42d3-b585-23919c359452.png)
4. Click left mouse to switch between the one-direction and bi-direction movement.
5. Click right mouse to switch between first-person and third-person mode.
