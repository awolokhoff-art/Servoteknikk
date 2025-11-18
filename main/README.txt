ELEVATOR CONTROL SIMULATOR
ALL-IN-ONE SERVOLAB
===========================================
 
DESCRIPTION
This file explains the setup and usage of the elevator control simulator developed for the All-In-One Servolab. The system simulates an elevator controlled via the Arduino Mega 2560, using the Arduino IDE.
 
 
SETUP AND INSTALLATION
-----------------------
1. Hardware:
   Connect the **Arduino Mega 2560** to the PC via a USB cable.
 
2. Software:
   Open the **"main.ino"** file in the Arduino IDE; header files will open automatically as long as they are in the same folder.
 
3. Configuration:
Use the **drop-down menu** in the upper left corner and select: **"COM3 'Controller Name'"** or other names resembling that. It depends on the physical port on you PC, but should resemble what is shown above.
 
4. Upload:
   Press the **"Upload"** button (the arrow icon) to compile and upload the code.
 
 


OPERATION
-----------------------
The system can receive commands from two locations: from **inside the elevator** (physical buttons) and from **outside** (via the serial monitor in Arduino IDE).
 
1. Calling from inside (Physical Buttons)
The physical buttons on the Servolab unit function as the buttons inside a elevator cabin.
   - Buttons **[0-7]** correspond to floors **[0-7]**.
   - Press a button to send the elevator to the desired floor.
   - Multiple floors can be pressed as the queuing code will take care of that.
 
2. Ordering from Outside (Serial Monitor)
The **Serial Monitor** is used to simulate the "Call" buttons in the hallway (outside the elevator).
   IMPORTANT:
   Open the Serial Monitor: Ensure the speed in the bottom right corner is set to 
   **115200 baud**.
 
   Command Syntax: **"XY"**
  --------------------------
   **X** = Floor Number. Valid numbers: **0 to 7**.
   **Y** = Direction. Valid letters:
         **u** = Up
         **d** = Down
 
 


USAGE EXAMPLES
-----------------
Situation 1: You are inside the elevator and want to go to the 4th floor.
Action:      Press the physical button **4** on the servo.
 
Situation 2: You are standing on the 5th floor and want to go DOWN.
Action:      Type **"5d"** in the Serial Monitor and press Enter.
 
Situation 3: You are standing on the 6th floor and want to go UP.
Action:      Type **"6u"** in the Serial Monitor and press Enter.
