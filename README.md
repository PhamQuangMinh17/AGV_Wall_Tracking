# AGV_Wall_Tracking
Simple wall-following automated guided vehicle for exploring basic of embedded system.

**Components:**

- Arduino Uno
- 3 Ultrasonic sensor HC-SR04: 1 front, 2 on right side of robot.
- 2 DC servo JGA25-370 + 2 wheels (with encoder).
- Motor Controlle Board L298n
- Robot Frame (self-design).
- 4 lithium batteries rechargable 18650 + battery holder.
- 2 step-down buck converters: 1 for powering up arduino UNo (with input Voltage of 5V) and 1 for adjusting voltage for Motor controller.
- 2 Switch: 1 for ON-OFF whole 1 system and 1 for ON-OFF Arduino Uno only.

**Usage:**
- Putting the AGV beside the worl so that the 2 ultrasonic sensors of the right-side of robot facing directly to the wall.
- After downloading code to Arduino and power up the AGV, it will start moving by continuously tracking the wall from the right side.
- AGV will turn left when there is an obstacle on the front.
