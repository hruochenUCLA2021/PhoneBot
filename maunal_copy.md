
Select Language​▼
Youtube
ROBOTIS
Community
GitHub

Edit on GitHub

    New XL430-W250 (Released in 2018)

    Old XL430-W250 (Discontinued)

Specifications
Item 	Specifications
MCU 	ARM CORTEX-M3 (72 [MHz], 32Bit)
Position Sensor 	Contactless absolute encoder (12Bit, 360 [°])
Maker : ams(www.ams.com), Part No : AS5601
Motor 	Cored
Baud Rate 	9,600 [bps] ~ 4.5 [Mbps]
Control Algorithm 	PID control
Resolution 	4096 [pulse/rev]
Operating Modes 	Velocity Control Mode
Position Control Mode (0 ~ 360 [°])
Extended Position Control Mode (Multi-turn)
PWM Control Mode (Voltage Control Mode)
Weight 	57.2 [g]
Dimensions (W x H x D) 	28.5 x 46.5 x 34 [mm]
Gear Ratio 	258.5 : 1
Stall Torque 	1.0 [N.m] (at 9.0 [V], 1.0 [A], 1.000 [Nm/A])
1.4 [N.m] (at 11.1 [V], 1.3 [A], 1.077 [Nm/A])
1.5 [N.m] (at 12.0 [V], 1.4 [A], 1.071 [Nm/A])
No Load Speed 	47 [rev/min] (at 9.0 [V])
57 [rev/min] (at 11.1 [V])
61 [rev/min] (at 12.0 [V])
Operating Temperature 	-5 ~ +72 [°C]
Input Voltage 	6.5 ~ 12.0 [V] (Recommended : 11.1 [V]) (lithium-ion battery 3 Cell)
Command Signal 	Digital Packet
Physical Connection 	TTL Multidrop Bus (5V Logic)
TTL Half Duplex Asynchronous Serial Communication
(8bit, 1stop, No Parity)
ID 	253 ID (0 ~ 252)
Feedback 	Position, Velocity, Load, Realtime tick, Trajectory, Temperature, Input Voltage, etc
Case Material 	Engineering Plastic
Gear Material 	Engineering Plastic
Standby Current 	52 [mA]


DANGER
(Ignoring these warnings may cause serious injury or death)

    Never place items containing water, flammables/open flames, or solvents near the product.
    Never place fingers, arms, toes, and other body parts near product during operation.
    Cease operation and remove power from the product if the product begins to emit strange odors, noises, or smoke.
    Keep product out of reach of children.
    Check input polarity before installing or energizing wiring or cables.


CAUTION
(Ignoring these warnings may cause mild injury or damage to the product)

    Always comply with the product’s offical operating environment specifications including input voltage, current, and operating temperature.
    Do not insert blades or other sharp objects during product operation.


ATTENTION
(Ignoring these warnings may cause minor injury or damage to the product)

    Do not disassemble or modify the product.
    Do not drop the product or apply strong impacts.
    Do not connect or disconnect DYNAMIXEL cables while power is being supplied.

Performance Graph

Looking for the same form factor?

		
	

	

	

	

		
	

	

	

	

		
		
	

	

		
	

	

		
	

	

	

	

		
	

	

	

	

	

	

	

	

		
	

	

NOTE : The given Stall torque rating for a servo is different from it’s continuous output rating, and may also differ from it’s expected real world performance.

Stall torque is the maximum momentary torque output the servo is capable of, an is generally how RC servos are measured. The Performance graph, or N-T curve, from the above graph is measured under conditions simulating a gradually increasing load.

Generally, the Maximum Torque shown through Performance Graph testing is less than the maximum Stall Torque.

The actual real world performance of the servo will generally be closer to the performance graph measurements, not the rated stall torque.

CAUTION - When supplying power:

    It is recommended to use a ROBOTIS controller or SMPS2DYNAMIXEL power adapter.

    Do not connect or disconnect DYNAMIXEL actuator cables while power is being supplied.

Control Table

The Control Table is a data structure used by DYNAMIXEL actuators to manage the state of the device. Users can read data registers to get information about the status of the device with Read Instruction Packets, and modify data registers to control the device with Write Instruction Packets.
Control Table, Data, Address

The Control Table is a structure that consists of multiple Data fields to store status or to control the device. Users can check current status of the device by reading a specific Data from the Control Table with Read Instruction Packets. WRITE Instruction Packets enable users to control the device by changing specific Data in the Control Table. The Address is a unique value when accessing a specific Data in the Control Table with Instruction Packets. In order to read or write data, users must designate a specific Address in the Instruction Packet. Please refer to DYNAMIXEL Protocol 2.0 for more details about Instruction Packets.

NOTE : Two’s complement is applied for the negative value. For more information, please refer to Two’s complement from Wikipedia.
Area (EEPROM, RAM)

The Control Table is divided into 2 Areas. Data in the RAM Area is reset to initial values when the power is reset(Volatile). On the other hand, data in the EEPROM Area is maintained even when the device is powered off(Non-Volatile).

Data in the EEPROM Area can only be modified if Torque Enable(64) is set to ‘0’ (Torque is OFF).
Size

The Size of data varies from 1 ~ 4 bytes depend on their usage. Please check the size of data when updating the data with an Instruction Packet. For data larger than 2 bytes will be saved according to Little Endian.
Access

The Control Table has two different access properties. ‘RW’ property stands for read and write access permission while ‘R’ stands for read only access permission. Data with the read only property cannot be changed by the WRITE Instruction. Read only property(‘R’) is generally used for measuring and monitoring purpose, and read write property(‘RW’) is used for controlling device.
Initial Value

Each data in the Control Table is restored to initial values when the device is turned on. Default values in the EEPROM area are initial values of the device (factory default settings). If any values in the EEPROM area are modified by a user, modified values will be restored as initial values when the device is turned on. Initial Values in the RAM area are restored when the device is turned on.
Control Table of EEPROM Area
Address 	Size(Byte) 	Data Name 	Access 	Initial
Value 	Range 	Unit
0 	2 	Model Number 	R 	1,060 	- 	-
2 	4 	Model Information 	R 	- 	- 	-
6 	1 	Firmware Version 	R 	- 	- 	-
7 	1 	ID 	RW 	1 	0 ~ 252 	-
8 	1 	Baud Rate 	RW 	1 	0 ~ 7 	-
9 	1 	Return Delay Time 	RW 	250 	0 ~ 254 	2 [μsec]
10 	1 	Drive Mode 	RW 	0 	0 ~ 13 	-
11 	1 	Operating Mode 	RW 	3 	0 ~ 16 	-
12 	1 	Secondary(Shadow) ID 	RW 	255 	0 ~ 252 	-
13 	1 	Protocol Type 	RW 	2 	1 ~ 2 	-
20 	4 	Homing Offset 	RW 	0 	-1,044,479 ~
1,044,479 	1 [pulse]
24 	4 	Moving Threshold 	RW 	10 	0 ~ 1,023 	0.229 [rev/min]
31 	1 	Temperature Limit 	RW 	72 	0 ~ 100 	1 [°C]
32 	2 	Max Voltage Limit 	RW 	140 	60 ~ 140 	0.1 [V]
34 	2 	Min Voltage Limit 	RW 	60 	60 ~ 140 	0.1 [V]
36 	2 	PWM Limit 	RW 	885 	0 ~ 885 	0.113 [%]
44 	4 	Velocity Limit 	RW 	265 	0 ~ 1,023 	0.229 [rev/min]
48 	4 	Max Position Limit 	RW 	4,095 	0 ~ 4,095 	1 [pulse]
52 	4 	Min Position Limit 	RW 	0 	0 ~ 4,095 	1 [pulse]
60 	1 	Startup Configuration 	RW 	0 	3 	-
63 	1 	Shutdown 	RW 	52 	- 	-
Control Table of RAM Area
Address 	Size(Byte) 	Data Name 	Access 	Initial
Value 	Range 	Unit
64 	1 	Torque Enable 	RW 	0 	0 ~ 1 	-
65 	1 	LED 	RW 	0 	0 ~ 1 	-
68 	1 	Status Return Level 	RW 	2 	0 ~ 2 	-
69 	1 	Registered Instruction 	R 	0 	0 ~ 1 	-
70 	1 	Hardware Error Status 	R 	0 	- 	-
76 	2 	Velocity I Gain 	RW 	1,000 	0 ~ 16,383 	-
78 	2 	Velocity P Gain 	RW 	100 	0 ~ 16,383 	-
80 	2 	Position D Gain 	RW 	4,000 	0 ~ 16,383 	-
82 	2 	Position I Gain 	RW 	0 	0 ~ 16,383 	-
84 	2 	Position P Gain 	RW 	640 	0 ~ 16,383 	-
88 	2 	Feedforward 2nd Gain 	RW 	0 	0 ~ 16,383 	-
90 	2 	Feedforward 1st Gain 	RW 	0 	0 ~ 16,383 	-
98 	1 	Bus Watchdog 	RW 	0 	1 ~ 127 	20 [msec]
100 	2 	Goal PWM 	RW 	- 	-PWM Limit(36) ~
PWM Limit(36) 	0.113 [%]
104 	4 	Goal Velocity 	RW 	- 	-Velocity Limit(44) ~
Velocity Limit(44) 	0.229 [rev/min]
108 	4 	Profile Acceleration 	RW 	0 	0 ~ 32,767
0 ~ 32,737 	214.577 [rev/min2]
1 [ms]
112 	4 	Profile Velocity 	RW 	0 	0 ~ 32,767 	0.229 [rev/min]
116 	4 	Goal Position 	RW 	- 	Min Position Limit(52) ~
Max Position Limit(48) 	1 [pulse]
120 	2 	Realtime Tick 	R 	- 	0 ~ 32,767 	1 [msec]
122 	1 	Moving 	R 	0 	0 ~ 1 	-
123 	1 	Moving Status 	R 	0 	- 	-
124 	2 	Present PWM 	R 	- 	- 	-
126 	2 	Present Load 	R 	- 	-1,000 ~ 1,000 	0.1 [%]
128 	4 	Present Velocity 	R 	- 	- 	0.229 [rev/min]
132 	4 	Present Position 	R 	- 	- 	1 [pulse]
136 	4 	Velocity Trajectory 	R 	- 	- 	0.229 [rev/min]
140 	4 	Position Trajectory 	R 	- 	- 	1 [pulse]
144 	2 	Present Input Voltage 	R 	- 	- 	0.1 [V]
146 	1 	Present Temperature 	R 	- 	- 	1 [°C]
147 	1 	Backup Ready 	R 	- 	0 ~ 1 	-
168 	2 	Indirect Address 1 	RW 	224 	64 ~ 661 	-
170 	2 	Indirect Address 2 	RW 	225 	64 ~ 661 	-
172 	2 	Indirect Address 3 	RW 	226 	64 ~ 661 	-
… 	… 	… 	… 	… 	- 	-
218 	2 	Indirect Address 26 	RW 	249 	64 ~ 661 	-
220 	2 	Indirect Address 27 	RW 	250 	64 ~ 661 	-
222 	2 	Indirect Address 28 	RW 	251 	64 ~ 661 	-
224 	1 	Indirect Data 1 	RW 	0 	0 ~ 255 	-
225 	1 	Indirect Data 2 	RW 	0 	0 ~ 255 	-
226 	1 	Indirect Data 3 	RW 	0 	0 ~ 255 	-
… 	… 	… 	… 	… 	- 	-
249 	1 	Indirect Data 26 	RW 	0 	0 ~ 255 	-
250 	1 	Indirect Data 27 	RW 	0 	0 ~ 255 	-
251 	1 	Indirect Data 28 	RW 	0 	0 ~ 255 	-
578 	2 	Indirect Address 29 	RW 	634 	64 ~ 661 	-
580 	2 	Indirect Address 30 	RW 	635 	64 ~ 661 	-
582 	2 	Indirect Address 31 	RW 	636 	64 ~ 661 	-
… 	… 	… 	… 	… 	- 	-
628 	2 	Indirect Address 54 	RW 	659 	64 ~ 661 	-
630 	2 	Indirect Address 55 	RW 	660 	64 ~ 661 	-
632 	2 	Indirect Address 56 	RW 	661 	64 ~ 661 	-
634 	1 	Indirect Data 29 	RW 	0 	0 ~ 255 	-
635 	1 	Indirect Data 30 	RW 	0 	0 ~ 255 	-
636 	1 	Indirect Data 31 	RW 	0 	0 ~ 255 	-
… 	… 	… 	… 	… 	- 	-
659 	1 	Indirect Data 54 	RW 	0 	0 ~ 255 	-
660 	1 	Indirect Data 55 	RW 	0 	0 ~ 255 	-
661 	1 	Indirect Data 56 	RW 	0 	0 ~ 255 	-

CAUTION : Protocol 1.0 does not support addresses greater than 256. Therefore, Indirect Address 29 ~ 56 and Indirect Data 29 ~ 56 can only be accessed with Protocol 2.0.
Control Table Description

CAUTION : Data in the EEPROM Area can only be modified when the value of Torque Enable(64) is set to ‘0’.
Model Number(0)

This address stores the model number of your DYNAMIXEL servo.
Decimal Value 	Hex Value 	Model Name
1060 	0x0424 	XL430-W250
Firmware Version(6)

This address stores the version number of the current firmware running on your DYNAMIXEL servo.
ID(7)

The ID assigned to a DYNAMIXEL actuator is a unique value used to identify a specific actuator through the DYNAMIXEL network. The numbers 0-253 (0xFD) can be assigned as an ID, with 254 (0xFE) reserved for use as the Broadcast ID that can send an Instruction Packet to all connected DYNAMIXEL servos simultaneously.

NOTE : IDs for every DYNAMIXEL connected to a single network must be unique. Shared ID numbers may cause communication failure.

NOTE : If an Instruction Packet ID is set to the Broadcast ID(0xFE), Status Packets will not be returned for READ or WRITE Instructions regardless of the configured Status Return Level (68). For more details, please refer to the Status Packet of the DYNAMIXEL Protocol 2.0 eManual page.
Baud Rate(8)

The Baud Rate(8) determines serial communication speed between a controller and DYNAMIXEL.
Value 	Baud Rate 	Margin of Error
7 	4.5M [bps] 	0.000 [%]
6 	4M [bps] 	0.000 [%]
5 	3M [bps] 	0.000 [%]
4 	2M [bps] 	0.000 [%]
3 	1M [bps] 	0.000 [%]
2 	115,200 [bps] 	0.000 [%]
1(Default) 	57,600 [bps] 	0.000 [%]
0 	9,600 [bps] 	0.000 [%]

NOTE : Less than 3% of the baud rate error margin will not affect to UART communication.

NOTE : For the stable communication with higher Baudrate using U2D2, configure USB Latency value to the lower.
USB Latency Setting
Return Delay Time(9)

When a DYNAMIXEL receives an Instruction Packet, it will return a Status Packet response after the configured Return Delay Time(9) has passed. The range of configurable values for this setting is 0 to 254 (0XFE) in units of 2 [μsec]. For instance, if the Return Delay Time(9) is set to ‘10’, a Status Packet will be returned 20[μsec] after an Instruction Packet is received.
Unit 	Value Range 	Description
2[μsec] 	0 ~ 254 	Default value ‘250’(500[μsec])
Maximum value: ‘508’[μsec]
Drive Mode(10)

Drive Mode contains several settings for adjusting the operating behavior of DYNAMIXEL servos, including automatic torque on, reverse movement, and movement profiles.
Bit 	Item 	Description 	 
Bit 7(0x80) 	- 	Unused, always ‘0’ 	 
Bit 6(0x40) 	- 	Unused, always ‘0’ 	 
Bit 5(0x20) 	- 	Unused, always ‘0’ 	 
Bit 4(0x10) 	- 	Unused, always ‘0’ 	 
Bit 3(0x08) 	Torque On by Goal Update 	[0] Movements will only be executed if Torque Enable(64) is set to ‘1’
[1] Movements will be executed regardless of the value of Torque Enable(64). If the value of Torque Enable(64) is ‘0’ and a command is given, Torque Enable(64) will be updated to ‘1’. 	 
Bit 2(0x04) 	Profile Configuration 	[0] Velocity-based Profile: Create Profiles based on movement Velocity
[1] Time-based Profile: Create Profiles based on time steps.
※ See What is the Profile 	 
Bit 1(0x02) 	- 	Unused, always ‘0’ 	 
Bit 0(0x01) 	Normal/Reverse Mode 	[0] Normal Mode: CCW(Positive), CW(Negative)
[1] Reverse Mode: CCW(Negative), CW(Positive) 	 

NOTE : Time-based Profile is available starting from firmware V42.

NOTE: Torque On by Goal Update is available starting from firmware V45.

NOTE : If the value of Bit 0(Normal/Reverse Mode) of Drive Mode(10) is set to 1, rotational direction is inverted.
Thus, Goal Position, Present Position will also have inverted directions. This feature can be very useful when configuring symmetrical joints, or similar mirrored DYNAMIXEL installations.
Operating Mode(11)
Value 	Operating Mode 	Description
1 	Velocity Control Mode 	This mode controls velocity. This mode behaves similarly to a standard DC motor, and is best suited to applications like drive wheels.
3(Default) 	Position Control Mode 	This mode controls position. This mode is identical to the Joint Mode from existing DYNAMIXEL. Operating position range is limited by the Max Position Limit(48) and the Min Position Limit(52). This mode is ideal for articulated robots that each joint rotates less than 360 degrees.
4 	Extended Position Control Mode(Multi-turn) 	This mode controls position. This mode is identical to the Multi-turn Position Control from existing DYNAMIXEL. 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for multi-turn wrists or conveyer systems or a system that requires an additional reduction gear. Note that Max Position Limit(48), Min Position Limit(52) are not used on Extended Position Control Mode.
16 	PWM Control Mode (Voltage Control Mode) 	This mode directly controls PWM output. (Voltage Control Mode)

NOTE : When the value of the Operating Mode(11) register changes, the Velocity PI(76, 78); Position PID(80, 82, 84); Feedforward(88, 90), will be reset according to the selected Operating Mode(11). Aside from this, the following registers will also be reset:

    Profile Velocity(112) and Profile Acceleration(108) will be set to ‘0’
    Goal PWM(100) is reset to the value of PWM Limit(36)

NOTE : PWM stands for Pulse Width Modulation, the signal that modulates voltage to control motors. It changes pulse width to control average supply voltage to the motor, and this technique is widely used in the motor control field.
PWM Control Mode is similar to the Wheel Mode of AX and RX series. Goal PWM(100) is used to control supply voltage for DYNAMIXELs in PWM Control Mode.

NOTE : Present Position(132) represents a 4 byte continuous range from -2,147,483,648 to 2,147,483,647 when Torque is turned off regardless of Operating Mode(11).
However, Present Position(132) will be reset to an absolute position value within one full rotation in the following cases:

    When the Operating Mode(11) is changed to Position Control Mode.
    When torque is turned on in Position Control Mode.
    When the actuator is turned on or when rebooted using a Reboot Instruction.

Note that a Present Position(132) value that has been reset to the absolute value within a single rotation will still be affected by the configured Homing Offset(20) value.
Secondary(Shadow) ID(12)

Secondary(Shadow) ID(12) allows the assignment of a secondary ID to a DYNAMIXEL.
The Secondary ID(12) can be used to group DYNAMIXELs and to synchronize their movement unlike the primary ID(7), which must be unique, any number of connected DYNAMIXELs may share a Secondary ID(12). There are several important differences between the primary ID(7) and Secondary ID(12):

    ID(7) has a greater priority than Secondary ID(12). If Secondary ID(12) and ID(7) are the same, the actuator will respond as if commands are sent only to it’s primary ID(7).
    The EEPROM area of the Control Table cannot be modified with control packets addressed to a Secondary ID(12).
    Status Packets will not be returned for commands sent to Secondary ID(12).
    If the value of Secondary ID(12) is 253 or higher, the Secondary ID function will be deactivated.

Values 	Description
0 ~ 252 	Activate Secondary ID function
253 ~ 255 	Deactivate Secondary ID function, Default value ‘255’
Secondary ID(12) Example

See the following Secondary ID(12) example for a demonstration. Note that the assigned ID(7) for the DYNAMIXELs in the example are ‘1’, ‘2’, ‘3’, ‘4’ and ‘5’.

    Set the Secondary ID of the five connected DYNAMIXELs to ‘5’
    Send a Write Instruction Packet(ID(7) = 1, LED(65) = 1).
    The DYNAMIXEL with ID(7)1’ turns on its LED and a Status Packet will be returned.
    Send Write Instruction Packet(ID(7) = 5, LED(65) = 1).
    All DYNAMIXELs turn on their LED, but only the actuator with an ID(7) of ‘5’ will return a Status Packet.
    Set the Secondary ID of all DYNAMIXELs to ‘100’.
    Send Write Instruction Packet(ID(7) = 100, LED(65) = 0).
    All DYNAMIXELs turn off their LED. As no DYNAMIXEL uses ID 100, no Status Packet will be returned.

Protocol Type(13)

Protocol Type(13) is used to set the communications protocol used by the DYNAMIXEL actuator.

WARNING : To modify the Protocol Type(13), use DYNAMIXEL Wizard 2.0 as R+ Manager 2.0 is not compatible with Protocol 1.0 products.

NOTE : Protocol 2.0 is more stable and safe for use than Protocol 1.0. Accessing some of the Control Table area might be denied if Protocol 1.0 is selected. Please refer to the Protocol 1.0 and Protocol 2.0 of e-Manual for more details about the protocol.

The following table lists available protocol types compatible with this model of DYNAMIXEL.
Value 	Description 	Compatible DYNAMIXEL
1 	DYNAMIXEL Protocol 1.0 	AX Series, DX Series, RX Series, EX Series, MX Series with Firmware below v39
2(default) 	DYNAMIXEL Protocol 2.0 	MX-28/64/106 with Firmware v39 or above, X Series, PRO Series

NOTE : Please refer to the Protocol Compatibility table to see a listing of supported protocols by product.
Homing Offset(20)

Homing Offset(20) allows users to adjust the actuator’s home position by adding the offset value is to the reported Present Position(132).

Present Position(132) = Actual Position + Homing Offset(20)
Unit 	Value Range
about 0.088 [°] 	-1,044,479 ~ 1,044,479
(-255 ~ 255[rev])

NOTE : In Position Control Mode(Joint Mode) when configured to rotate less than 360 degrees, any invalid Homing Offset(20) values will be ignored(valid range : -1,024 ~ 1,024).

WARNING : Even if Drive Mode(10) is set to Reverse Mode, the sign of the Homing Offset(20) value is not reversed.
Moving Threshold(24)

The Moving Threshold(24) is used to determine whether the DYNAMIXEL is considered to be in motion or not. When the absolute value of the Present Velocity(128) is greater than the configured Moving Threshold(24), the DYNAMIXEL is considered in motion and the value of Moving(122) is set to ‘1’.
Unit 	Range 	Description
about 0.229 rpm 	0 ~ 1,023 	All velocity related Data uses the same unit scale
Temperature Limit(31)

The Temperature Limit(31) is used to configure an upper limit on DYNAMIXEL operating temperature.
When the Present Temperature(146) is greater than the Temperature Limit(31), the Overheating Error Bit(0x04) and Alert Bit(0x80) in the Hardware Error Status(70) register will be set. If the Overheating Error Bit(0x04) is configured in Shutdown(63), Torque Enable(64) will be set to ‘0’ (Torque OFF). See Shutdown(63) for more detailed information.
Unit 	Value Range 	Description
About 1° 	0 ~ 100 	0 ~ 100°

CAUTION : Do not set this value higher than it’s default. In the case that a DYNAMIXEL triggers a temperature shutdown alarm (Overheating Error Bit(0x04)), let it cool for 20 minutes or more before resuming use to prevent damage to the actuator.
Min/Max Voltage Limit(34, 32)

The Min Voltage Limit(32) and Max Voltage Limit(34) are used to configure the range of acceptable input voltages. When the Present Input Voltage(144) exits the range between Max Voltage Limit(32) and Min Voltage Limit(34), the Input Voltage error Bit(0x10) in the Hardware Error Status(70) will be set, and the returned Status Packet will contain an Alert Bit(0x80) in the Error field.
If the Input Voltage Error Bit(0x10) in the Shutdown(63) register is set, Torque Enable(64) will be set to ‘0’(Torque OFF).
For more details, please refer to Shutdown(63).
Unit 	Value Range 	Description
About 0.1 [V] 	65 ~ 140 	6.5 ~ 14.0 [V]
PWM Limit(36)

The PWM Limit(36) is used to configure the maximum allowable PWM output. Goal PWM(100) can’t be set to a values exceeding the PWM Limit(36). Additionaly, the PWM Limit(36) is used in all operating mode as an absolute limit to the motor’s final output PWM. For more details, please refer to the Gain section of each operating mode.
Unit 	Range
about 0.113 [%] 	0(0 [%]) ~ 885(100 [%] )
Velocity Limit(44)

The Velocity Limit(44) is used to limit the maximum acceptable value for the Goal Velocity(104). For more details, see Goal Velocity(104).
Unit 	Value Range
0.229rpm 	0 ~ 1,023
Min/Max Position Limit(52, 48)

The Min and Max Position Limit(48, 52) is used to configure the maximum movement range in Position Control Mode within a single rotation(0 ~ 4,095).
Provided Goal Position(116) values must be within the configured position limit range.
Unit 	Value Range
0.088 [°] 	0 ~ 4,095(1 rotation)

NOTE : Max Position Limit(48) and Min Position Limit(52) are only used in Position Control Mode within a single turn. The angle limits set through this control table item do not apply in Extended Position Control Mode.
Startup Configuration(60)

Startup Configuration(60) allows users to configure several settings to apply on actuator startup. See the following table for information on available settings.
Bit 	Item 	Description
Bit 7(0x80) 	- 	Unused, always ‘0’
Bit 6(0x40) 	- 	Unused, always ‘0’
Bit 5(0x20) 	- 	Unused, always ‘0’
Bit 4(0x10) 	- 	Unused, always ‘0’
Bit 3(0x08) 	- 	Unused, always ‘0’
Bit 2(0x04) 	- 	Unused, always ‘0’
Bit 1(0x02) 	RAM Restore 	[0] Deactivate RAM restoration on startup.
[1] On startup, use saved backup data to restore RAM configuration.
Bit 0(0x01) 	Startup Torque On 	[0] Torque Off on startup (Torque Enable(64) is set to 0)
[1] Torque On on startup (Torque Enable(64) is set to 1).

NOTE: Startup Configuration is available from firmware V45.

NOTE: For more details about restoring the RAM area, see Restoring RAM Area.
Shutdown(63)

DYNAMIXEL servos can protect themselves by detecting dangerous situations that may occur during operation. This register allows user to configure which of these error states causes a safety shutdown. Each Bit is inclusively processed with ‘OR’ logic, allowing multiple options to be selected. For instance, when Shutdown(63) is set to ‘0x05’ (binary : 00000101), the DYNAMIXEL will shutdown in response to both both an Input Voltage Error(binary : 00000001) and Overheating Error(binary : 00000100). If those errors are detected, Torque Enable(64) is cleared to ‘0’ and the motor’s output will be set to 0 [%].

A REBOOT is the only method to reset Torque Enable(64) to ‘1’(Torque ON) after a shutdown has been triggered.
Bit 	Item 	Description 	 
Bit 7 	- 	Unused, Always ‘0’ 	 
Bit 6 	- 	Unused, Always ‘0’ 	 
Bit 5 	Overload Error(default) 	Detects that persistent load that exceeds maximum output 	 
Bit 4 	Electrical Shock Error(default) 	Detects electric shock on the circuit or insufficient power to operate the motor 	 
Bit 3 	Motor Encoder Error 	Detects malfunction of the motor encoder 	 
Bit 2 	Overheating Error(default) 	Detects that internal temperature exceeds the configured operating temperature 	 
Bit 1 	- 	Unused, Always ‘0’ 	 
Bit 0 	Input Voltage Error 	Detects that input voltage exceeds the configured operating voltage 	 

NOTE :

    If Shutdown occurs, the indicator LED will flicker every second. (Firmware v41 or above)
    If a Shutdown occurs, reboot the device to restore functionality.
        H/W REBOOT : Turn off and turn on the power to the actuator.
        S/W REBOOT : Transmit REBOOT Instruction (For more details, refer to the Reboot section of the e-Manual.)

Torque Enable(64)

Torque Enable(64) is used to enable or disable the torque for the DYNAMIXEL’s internal motor. Setting Torque Enable(64) to ‘1’ will enable output Torque and all Data in the EEPROM area will be locked. Setting the value to ‘0’ will disable torque and unlock the EEPROM.
Value 	Description
0(Default) 	Torque Off, EEPROM unlocked
1 	Torque On, EEPROM locked

NOTE : Present Position(132) will be reset when the Operating Mode(11) or Torque Enable(64) settings are updated. For more details, please refer to the Homing Offset(20) and Present Position(132) control table items.
LED(65)

LED(65) controls the state of the indicator LED located on the back of the DYNAMIXEL actuator.
Bit 	Description
0(Default) 	Turn OFF the LED
1 	Turn ON the LED

NOTE : The LED is also used to indicate various statuses of the DYNAMIXEL actuator, refer to the following chart for more information.
Status 	LED Representation
Booting 	LED blinks once
Factory Reset 	LED blinks quickly 4 times
Shutdown Error 	LED blinks continuously
Bootloader Mode 	LED on continuously
Status Return Level(68)

The Status Return Level (68) is used to set the DYNAMIXEL’s response policy when an instruction packet is received.
Value 	Responding Instructions 	Description
0 	PING Instruction 	Returns a Status Packet for PING Instructions only
1 	PING Instruction
READ Instruction 	Returns a Status Packet for PING and READ Instructions
2 	All Instructions 	Returns a Status Packet for all Instructions

NOTE : If the Instruction Packet ID is set to the Broadcast ID(0xFE), a Status Packet will not be returned for READ or WRITE Instructions regardless of Status Return Level (68). For more details, please refer to the Status Packet section of the DYNAMIXEL Protocol 2.0 page.
Registered Instruction(69)

Indicates whether an instruction has been registered by the Reg Write Instruction and is ready for execution. Following the execution of a registered action the Registered Instruction (69) value will automatically be updated to 0.
Value 	Description
0 	No instruction registered by REG_WRITE.
1 	Instruction registered by REG_WRITE.
Hardware Error Status(70)

This register stores any active hardware error status conditions. For more details, please refer to the Shutdown(48) control table item.
Velocity PI Gain(76, 78)

The Velocity PI Gains(76, 78) are the configurable gain settings for the DYNAMIXEL PID controller in Velocity Mode.
  	Controller Gain 	Conversion Equations 	Range 	Description
Velocity I Gain(76) 	KVI 	KVI = KVI(TBL) / 65,536 	0 ~ 16,383 	I Gain
Velocity P Gain(78) 	KVP 	KVP = KVP(TBL) / 128 	0 ~ 16,383 	P Gain

When an instruction transmitted from the user is received by a DYNAMIXEL servo, it takes the following steps to generate a trajectory and execute a motion.

    An Instruction from the user is transmitted via the DYNAMIXEL bus, then written to Goal Velocity(104).
    Goal Velocity(104) is converted to planned velocity trajectory by the configured Profile Acceleration(108).
    The desired velocity trajectory is stored at Velocity Trajectory(136).
    The PI controller calculates PWM output for the motor based on the desired velocity trajectory.
    Goal PWM(100) sets a limit on the calculated PWM output and decides the final PWM value.
    The final PWM value is applied to the motor through an Inverter, and the horn of the DYNAMIXEL is driven.
    Results are stored at Present Position(132), Present Velocity(128), Present PWM(124) and Present Load(126).

The below figure is a block diagram describing the velocity controller in Velocity Control Mode.

NOTE : Ka Anti-windup Gain and β are conversion coefficients of position and velocity that cannot be modified by users. For more information about PID controllers in general, please refer to the PID Controller article on Wikipedia.
Position PID Gain(80, 82, 84), Feedforward 1st/2nd Gains(88, 90)

The Position PID Gains(76, 78) are the configurable gain settings for the DYNAMIXEL PID controller in Position Control Mode and Extended Position Control Mode.
  	Controller Gain 	Conversion Equations 	Range 	Description
Position D Gain(80) 	KPD 	KPD = KPD(TBL) / 16 	0 ~ 16,383 	D Gain
Position I Gain(82) 	KPI 	KPI = KPI(TBL) / 65,536 	0 ~ 16,383 	I Gain
Position P Gain(84) 	KPP 	KPP = KPP(TBL) / 128 	0 ~ 16,383 	P Gain
Feedforward 2nd Gain(88) 	KFF2nd 	KFF2nd(TBL) / 4 	0 ~ 16,383 	Feedforward Acceleration Gain
Feedforward 1st Gain(90) 	KFF1st 	KFF1st(TBL) / 4 	0 ~ 16,383 	Feedforward Velocity Gain

When an instruction transmitted from the user is received by a DYNAMIXEL servo, it takes the following steps to generate a trajectory and execute a motion.

    An Instruction from the user is transmitted via the DYNAMIXEL bus, then registered to Goal Position(116).
    Goal Position(116) is converted to desired position trajectory and desired velocity trajectory by Profile Velocity(112) and Profile Acceleration(108).
    The desired position trajectory and desired velocity trajectory is stored in the Position Trajectory(140) and Velocity Trajectory(136) registers.
    The Feedforward and PID controller calculate PWM output for the motor based on desired trajectories.
    Goal PWM(100) is used as a limit on the calculated PWM output and decides the final output PWM value.
    The final PWM value is applied to the motor through an inverter, and the horn of the DYNAMIXEL is driven.
    Results are stored at Present Position(132), Present Velocity(128), Present PWM(124) and Present Load(126).

The figure below is a block diagram describing the position controller in Position Control Mode and Extended Position Control Mode.

NOTE:

    In PWM Control Mode, both the internal PID controller and Feedforward controller are deactivated while Goal PWM(100) is used to directly control the voltage to the motor.
    Ka is an Anti-windup Gain that cannot be modified by users.
    For more details about PID controllers and Feedforward controllers in general, please refer to the PID Controller and Feed Forward Wikipedia articles.

Bus Watchdog(98)

The Bus Watchdog(98) is a fail-safe system used to stop DYNAMIXEL motion if communication between the controller and DYNAMIXEL (RS-485, TTL) is disconnected.
  	Values 	Description
Range 	0 	Deactivate Bus Watchdog Function, Clear Bus Watchdog Error
Range 	1 ~ 127 	Activate Bus Watchdog (Unit: 20 [msec])
Range 	-1 	Bus Watchdog Error Active

The Bus Watchdog function monitors the interval of communications between the controller and DYNAMIXEL when Torque Enable(64) is ‘1’(Torque ON).
If the measured communication interval is larger than the configured value of Bus Watchdog(98), the DYNAMIXEL will stop. Bus Watchdog(98) will be changed to ‘-1’ and a Bus Watchdog Error will be activated. When a Bus Watchdog Error screen is active, all Goal Values (Goal PWM(100), Goal Velocity(104), Goal Position(116)) will be changed to read-only-access until the error has been cleared. If the value of Bus Watchdog(98) is changed to ‘0’, active Bus Watchdog Errors will be cleared.
Bus Watchdog (98) Example

The following is an example of the operation of the Bus Watchdog failsafe.

    After setting the Operating Mode(11) to speed control mode, change Torque Enable(64) to ‘1’.
    If ‘50’ is written to Goal Velocity(104), the DYNAMIXEL will rotate in a CCW direction.
    Change the value of Bus Watchdog(98) to ‘100’ (2,000 [ms]) to activate the Bus Watchdog function.
    If no instruction packet is received for 2,000 [ms], the DYNAMIXEL will stop. When it stops, the Profile Acceleration(108) and Profile Velocity(112) are set to ‘0’.
    The value of Bus Watchdog(98) changes to ‘-1’ (Bus Watchdog Error). At this time, the access to Goal Values will be changed to read-only.
    If ‘150’ is written to Goal Velocity(104), a Data Range Error will be returned via Status Packet.
    If the value of Bus Watchdog(98) is changed to ‘0’, the Bus Watchdog Error will be cleared.
    If “150” is written to Goal Velocity(104), the DYNAMIXEL will rotate in a CCW direction.

Goal PWM(100)

When the Operating Mode(11) is set to PWM Control Mode, both the internal PID and Feedforward controllers will be deactivated and Goal PWM(100) is used to directly control the supplied output voltage to the servo’s motor. When set to a different Operating Mode(11), Goal PWM(100) is used as a final limiter on the output PWM value only. Read Position PID Gain(80, 82, 84), Feedforward 1st/2nd Gains(88, 90) or Velocity PI Gain(76, 78) for more information on how Goal PWM (100) works with configurable gain settings.
Unit 	Range
about 0.113 [%] 	-PWM Limit(36) ~ PWM Limit(36)

NOTE: Goal PWM(100) can not exceed the configured PWM Limit(36).
Goal Velocity(104)

Goal Velocity(104) is used to to set the target velocity when in the Velocity Control Mode Operating Mode(11). Goal Velocity(104) is not used to limit moving velocity in any operating modes.
Unit 	Value Range
0.229 rpm 	-Velocity Limit(44) ~ Velocity Limit(44)

NOTE: Goal Velocity(104) can not exceed the configured Velocity Limit(44).

NOTE : The maximum velocity and maximum torque of DYNAMIXEL is affected by the supplied voltage. If the input voltage changes, so does the maximum velocity and torque. This manual assumes operation with the recommended input voltage.

NOTE : If Profile Acceleration(108) and Goal Velocity(104) are modified simultaneously, the newly modified Profile Acceleration(108) will be used to process the updated Goal Velocity(104).
Profile Acceleration(108)

When the Drive Mode(10) is Velocity-based Profile, Profile Acceleration(108) sets the maximum allowable acceleration of the generated Profile.
When the Drive Mode(10) is Time-based Profile, Profile Acceleration(108) sets the allowed acceleration time of the generated Profile.
The Profile Acceleration(108) is used in all control modes except Current Control Mode and PWM Control Mode.

For more detailed information, see What is the Profile
Velocity-based Profile 	Values 	Description
Unit 	214.577 [rev/min2] 	Sets maximum acceleration of the Profile
Range 	0 ~ 32767 	‘0’ represents an infinite maximum acceleration
Time-based Profile 	Values 	Description
Unit 	1 [msec] 	Sets allowed acceleration time of the Profile
Range 	0 ~ 32737 	‘0’ represents an infinite acceleration time(‘0 [msec]’).
Profile Acceleration(108) can not exceed 50% of the configured Profile Velocity (112) value.

NOTE : Time-based Profiles are available starting from firmware version 42.
Profile Velocity(112)

When the Drive Mode(10) is Velocity-based Profile, Profile Velocity(112) sets the maximum allowable velocity of the generated Profile.
When the Drive Mode(10) is Time-based Profile, Profile Velocity(112) sets the total execution time of the generated Profile. Profile Velocity(112) is only applied in the Position Control Mode or Extended Position Control Mode Operating Mode(11).

For more detailed information, see What is the Profile.

NOTE: Velocity Control Mode only uses Profile Acceleration(108)
Velocity-based Profile 	Values 	Description
Unit 	0.229 [rev/min] 	Sets the maximum velocity of the Profile
Range 	0 ~ 32767 	‘0’ represents an infinite velocity
Time-based Profile 	Values 	Description
Unit 	1 [msec] 	Sets the time span for the Profile
Range 	0 ~ 32737 	‘0’ represents an infinite velocity.
Profile Acceleration(108) can not exceed 50% of the configured Profile Velocity (112) value.

NOTE : Time-based Profiles are available from firmware V42.
Goal Position(116)

Goal Position(116) is used to set the actuator’s desired output position.

Mode 	Values 	Description
Position Control Mode 	Min Position Limit(52) ~ Max Position Limit(48) 	Initial Value : 0 ~ 4,095
Extended Position Control Mode 	-1,048,575 ~ 1,048,575 	-256[rev] ~ 256[rev]
Unit 	Description
0.088 [deg/pulse] 	1[rev] : 0 ~ 4,095

NOTE : Profile Velocity(112) and Profile Acceleration(108) are active under the following conditions:

    When the Operating Mode(11) is Position Control Mode, Profile Velocity(112) and Profile Acceleration(108) are used to create a new profile when the Goal Position(116) is updated.
    When the Operating Mode(11) is Velocity Control Mode, Profile Acceleration(108) is used to create a new profile when Goal Velocity(104) is updated.

NOTE : When turning off the power supply or changing the Operating Mode to Extended Position Control Mode, the value of Present Position is reset to the absolute position value within a single turn.

NOTE : Present Position(132) represents a 4 byte continuous range from -2,147,483,648 to 2,147,483,647 when Torque is turned off regardless of Operating Mode(11).
However, Present Position(132) will be reset to an absolute position value within one full rotation in the following cases:

    When the Operating Mode(11) is changed to Position Control Mode.
    When torque is turned on in Position Control Mode.
    When the actuator is turned on or when rebooted using a Reboot Instruction.

Note that a Present Position(132) value that has been reset to the absolute value within a single rotation will still be affected by the configured Homing Offset(20) value.
Realtime Tick(120)

Realtime Tick(120) tracks the uptime of the current DYNAMIXEL actuator. This register begins counting at 0 when the actuator is powered on.
Unit 	Value Range 	Description
1 ms 	0 ~ 32,767 	The value resets to ‘0’ when it exceeds 32,767
Moving(122)

The Moving(122) register indicates whether the connected DYNAMIXEL is in motion or not.
If the absolute value of Present Velocity(128) is greater than the configured Moving Threshold(24), Moving(122) is set to ‘1’, indicating the actuator is currently in motion. Otherwise, it will be cleared to ‘0’. However, Moving(122) will always be set to ‘1’ regardless of Present Velocity(128) while a movement Profile is in progress following an updated Goal Position(116) instruction.
Value 	Description
0 	Movement is not detected
1 	Movement is detected, or a Profile is in progress(Goal Position(116) instruction has been updated)
Moving Status(123)

Moving Status(123) is a collection of bits that provide additional information about the DYNAMIXEL’s current motion status.
Bit 	Value 	Information 	Description
Bit 7 	X 	- 	Reserved for future use
Bit 6 	X 	- 	Reserved for future use
Bit 4
Bit 5 	11
10
01
00 	Velocity Profile 	11 : Trapezoidal Profile
10 : Triangular Profile
01 : Rectangular Profile
00 : Profile not used(Step)
Bit 3 	0 or 1 	Following Error 	Whether or not the DYNAMIXEL is following the desired position trajectory
0 : Following
1 : Not following
Bit 2 	X 	- 	Reserved for future use
Bit 1 	0 or 1 	Profile Ongoing 	A Profile is in progress following a Goal Position(116) instruction update
0 : Profile completed
1 : Profile in progress
Bit 0 	0 or 1 	In-Position 	Whether or not the DYNAMIXEL has arrived to the desired Goal Position(116)
0 : Not arrived
1 : Arrived

Following Error(0x08) and In-Position(0x01) are available when using Position Control Mode, and Extended Position Control Mode.

For more details about these operating modes, see Operating Mode(11). NOTE : A Triangular velocity profile is when a configured Rectangular velocity profile cannot reach the configured Profile Velocity(112).

NOTE : The In-Position bit will be set when the positional deviation is smaller than a predefined value when in Position related control modes.
Present PWM(124)

This value indicates the present PWM duty being passed to the motor. For more details, please refer to Goal PWM(100).
Present Load(126)

Present Load reports an estimate of the current load applied to the motor’s output.
Unit 	Value Range 	Description
0.1% 	-1,000 ~ 1,000 	Positive(CCW Load), Negative(CW Load)

NOTE : The Present load is an inferred value based on the internal output value; and is not measured using a torque sensor, etc. Therefore, it may be inaccurate for precise measuring of weight or torque. It is recommended to use it only for predicting and monitoring the direction and size of forces being applied to the servo.
Present Velocity(128)

Present Velocity reports the current output speed of the actuator. For more details, please refer to Goal Velocity(104).
Present Position(132)

The Present Position(132) indicates the present actuator Position. For more details, see Goal Position(116).

NOTE : Present Position(132) represents a 4 byte continuous range from -2,147,483,648 to 2,147,483,647 when Torque is turned off regardless of Operating Mode(11).
However, Present Position(132) will be reset to an absolute position value within one full rotation in the following cases:

    When the Operating Mode(11) is changed to Position Control Mode.
    When torque is turned on in Position Control Mode.
    When the actuator is turned on or when rebooted using a Reboot Instruction.

Note that a Present Position(132) value that has been reset to the absolute value within a single rotation will still be affected by the configured Homing Offset(20) value.
Velocity Trajectory(136)

This register stores the current target velocity for the active trajectory created by the current Profile. The specific values and behavior of this register may change based on the active operating mode. For more details, please refer to Profile Velocity(112).

    Velocity Control Mode : When the generated Profile reaches it’s endpoint, Velocity Trajectory(136) becomes equal to Goal Velocity(104).
    Position Control Mode, Extended Position Control Mode : Velocity Trajectory is used to create the Position Trajectory(140) used for the movement. When the Profile reaches an endpoint, Velocity Trajectory(136) is set to ‘0’.

Position Trajectory(140)

The Position Trajectory(140) is the desired movement trajectory created by the active Profile.
Position Trajectory(140) is used only when the Operating Mode(11) is Position Control Mode, or Extended Position Control Mode
For more details, see What is the Profile.
Present Input Voltage(144)

The Present Input Voltage(144) reports the present voltage level being supplied to the actuator. For more details, see the Max/Min Voltage Limit(32, 34).
Present Temperature(146)

The Present Temperature(144) reports the present operating tempurature of the actuator. For more details, see Temperature Limit (31).
Backup Ready(147)

The value in this address indicates whether a saved backup of the control table exists in the actuator’s memory. For more information see Control Table Backup Packet.
Value 	Description
0 	There is no saved backup data
1 	Saved backup data exists.

NOTE
Backup Ready is available starting from firmware V45.
See Backup and Restore for more details.
Indirect Address, Indirect Data

The Indirect Address registers allow the assignment of other RAM area control table items to registers in the Indirect Data region. This allows users to map multiple control table items needed for their application to sequential memory addresses to improve Instruction Packet efficiency. If a specific address is allocated an Indirect Address, the corresponding Indirect Data register inherits all the features and properties of the assigned control table address. These properties include Size (Byte length), value range, and Access properties (Read Only, Read/Write). For instance, if 65 (LED) is allocated to Indirect Address 1(168), Indirect Data 1(208) will take on the properties of the LED(65) register.

NOTE: Only addresses in the RAM area (Address 64 ~ 227) of the control table may be assigned to Indirect Addresses.
Indirect Address Range 	Description
64 ~ 661 	EEPROM addresses can’t be assigned an Indirect Address
Indirect Address and Indirect Data Examples

Example 1 Allocating LED(65) to Indirect Data 1(224).

    Change the value of Indirect Address 1(168) to ‘65’ which is the address used for LED control.
    Set Indirect Data 1(224) to ‘1’, LED(65) also changes to ‘1’ and the LED is turned on.
    Set Indirect Data 1(224) to ‘0’, LED(65) also changes to ‘0’ and the LED is turned off.

Example 2 Allocating Goal Position(116) to Indirect Data 2(225), 4 sequential bytes have to be allocated.

    Indirect Address 2(170) : change the value to ‘116’ which is the first address of Goal Position.
    Indirect Address 3(172) : change the value to ‘117’ which is the second address of Goal Position.
    Indirect Address 4(174) : change the value to ‘118’ which is the third address of Goal Position.
    Indirect Address 5(176) : change the value to ‘119’ which is the fourth address of Goal Position.
    Set 4 byte value ‘1,024’ to Indirect Data 2 : Goal Position(116) also becomes ‘1024’ and DYNAMIXEL moves.

NOTE : In order to allocate Data in the Control Table longer than 2[byte] to Indirect Address, all address must be allocated to Indirect Address like the above Example 2.

NOTE : Indirect Address 29 ~ 56 and Indirect Data 29 ~ 56 can only be accessed with Protocol 2.0.
How to Assemble
Horn Assembly
Idler Horn Assembly

An idler horn is required for installation of hinge frame accessories.

Additionally, the hollow shaft of an installed idler horn provides a neat cable wiring solution.

HowTo_Idler_Assembly

    Hollow shaft cable assembly precautions

CAUTION: To ensure correct DYNAMIXEL-X series cable assembly untangle the DYNAMIXEL cable prior to insertion through the hollow shaft. Do not assemble the back case with a tangled cable, tangled cables may be crushed by the case and cause communication errors or damage to your DYNAMIXEL hardware.
Frame Assembly
How To Use Spacer Ring

To prevent damage to your frames during assembly, use the included spacer rings to fill the gaps between assembled frames and your DYNAMIXEL case.

HowTo_SpacerRing
Precaution of Frame and Horn Assembly

WARNING: Before assembling your DYNAMIXEL accessories, ensure that all screws and bolts are the correct length by considering the depth of your DYNAMIXEL’s mounting points. If the length of screw is larger than the depth of the mounting point your frame or DYNAMIXEL may be damaged during assembly.

Warn_HornAssembly

NOTE: Information regarding size and depth of DYNAMIXEL servo mounting points can be found in the Drawings section of the product’s eManual page.
Hinge Frame Assembly

DYNAMIXEL hinge frames are assembled by attaching them to the idler and output horn of your servo.

Hinge_Assembly

    FR12-H101K

Hinge_Assembly

    FR12-H104K

Hinge_Assembly

    FR12-H104K (Back Mount, compatible with DYNAMIXEL XL & XC Series)

Hinge_Assembly

    FR12-H103GM

NOTE: Information regarding size and depth of DYNAMIXEL servo mounting points can be found in the Drawings section of the product’s eManual page.

NOTE: An idler horn is required for the installation of DYNAMIXEL hinge frames. See the Idler Horn Assembly instructions for more information.

WARNING: During hinge assembly, ensure that all screws are the proper length before installation. See Frame and Horn Assembly Precautions for more information.
Side Frame Assembly

DYNAMIXEL side frames are assembled by attaching them to the mounting points on the sides of your DYNAMIXEL actuator.

Side_Assembly_Side

    FR12-S101K

Side_Assembly_Bottom

    FR12-S102K

NOTE: Information regarding size and depth of DYNAMIXEL servo mounting points can be found in the Drawings section of the product’s eManual page.

NOTE: Use spacer rings to protect assembled DYNAMIXEL frames. See How To Use Spacer Rings for more information.
Gripper Assembly

etc_gripper_assembly

    FR12-G101GM (FR12-E170 + FR12-E171)

NOTE: Information regarding size and depth of DYNAMIXEL servo mounting points can be found in the Drawings section of the product’s eManual page.

NOTE: Use spacer rings to protect assembled DYNAMIXEL frames. See How To Use Spacer Rings for more information.
Frame Combination

Hinge and side frames can be combined in a variety of ways to provide complex mounting options.

Frame_Example

NOTE: Find more detailed screw information on frame to frame assembly in the Drawings section.
Custom Frame Assembly

Custom made DYNAMIXEL frames can also be installed by following the instructions below.
Front (Flat Head Screw)

    Step 1

    ETC_FrameAssembly

    Step 2

    ETC_FrameAssembly

NOTE: The example frame included in the image is not available for sale.

NOTE: Use spacer rings to protect assembled DYNAMIXEL frames see How To Use Spacer Rings for more information.

NOTE: Information regarding size and depth of DYNAMIXEL servo mounting points can be found in the Drawings section of the product’s eManual page.
Front (Pan Head Screw)

    Step 1

    ETC_FrameAssembly

    Step 2

    ETC_FrameAssembly

NOTE: The example frame included in the image is not available for sale.

NOTE: Use spacer rings to protect assembled DYNAMIXEL frames see How To Use Spacer Rings for more information.

NOTE: Information regarding size and depth of DYNAMIXEL servo mounting points can be found in the Drawings section of the product’s eManual page.
Side

ETC_FrameAssembly

WARNING: Before assembling your DYNAMIXEL accessories, ensure that all screws and bolts are the correct length by considering the depth of your DYNAMIXEL’s mounting points. If the length of screw is larger than the depth of the mounting point your frame or DYNAMIXEL may be damaged during assembly.

3mm_Mount_Deep_Warning

NOTE: The example frame included in the image is not available for sale.

NOTE: Information regarding size and depth of DYNAMIXEL servo mounting points can be found in the Drawings section of the product’s eManual page.
Bottom

ETC_FrameAssembly

WARNING: Before assembling your DYNAMIXEL accessories, ensure that all screws and bolts are the correct length by considering the depth of your DYNAMIXEL’s mounting points. If the length of screw is larger than the depth of the mounting point your frame or DYNAMIXEL may be damaged during assembly.

3mm_Mount_Deep_Warning

NOTE: The example frame included in the image is not available for sale.

NOTE: Use spacer rings to protect assembled DYNAMIXEL frames see How To Use Spacer Rings for more information.

NOTE: Information regarding size and depth of DYNAMIXEL servo mounting points can be found in the Drawings section of the product’s eManual page.
Reference

NOTE Compatibility Guide
Harness Compatibility
What is the Profile

The Profile is a generated movement trajectory intended to reduce vibration, noise and load of the motor by dynamically changing velocity and acceleration during movements. DYNAMIXEL servos provide 3 different types of Profile:

Profiles are usually selected according to the combination of Profile Velocity(112) and Profile Acceleration(108).

When given a new Goal Position(116), the DYNAMIXEL’s profile settings creates a desired velocity trajectory based on present movement velocity. When a DYNAMIXEL receives an updated Goal Position(116) while it is moving toward the previous Goal Position(116), velocity is adjusted smoothly to match the new desired velocity trajectory.
The following explains how the Profile processes Goal Position(116) instructions in Position Control mode, and Extended Position Control Mode.

    An Instruction from the user is transmitted via the DYNAMIXEL bus, then registered to Goal Position(116) (If Velocity-based Profile is selected).
    Acceleration time(t1) is calculated based on Profile Velocity(112) and Profile Acceleration(108).
    The type of Profile is decided based on Profile Velocity(112), Profile Acceleration(108) and total travel distance(ΔPos, the distance difference between desired position and present position).
    The selected Profile type is stored at Moving Status(123).
    The DYNAMIXEL is driven by the calculated desired trajectory from the Profile.
    The desired velocity trajectory and desired position trajectory from the Profile are stored at Velocity Trajectory(136) and Position Trajectory(140) respectively.

Condition 	Types of Profile
VPRFL(112) = 0 	Profile not used
(Step Instruction)
(VPRFL(112) ≠ 0) & (APRF(108) = 0) 	Rectangular Profile
(VPRFL(112) ≠ 0) & (APRF(108) ≠ 0) 	Trapezoidal Profile

NOTE : Velocity Control Mode only uses Profile Acceleration(108). Step and Trapezoidal Profiles are supported. Acceleration time(t1) can be calculated according to the equation below.

Velocity-based Profile : t1 = 64 * {Profile Velocity(112) / Profile Acceleration(108)}
Time-based Profile : t1 = Profile Acceleration(108)

NOTE : If Time-based Profile is selected, Profile Velocity(112) is used to set the time span of the Profile(t3), while Profile Acceleration(108) sets allowed accelerating time(t1) in millisecond[ms]. Profile Acceleration(108) will not exceed 50% of the configured Profile Velocity(112) value.
Certifications

Please inquire us for information regarding unlisted certifications.
FCC

Note: This equipment has been tested and found to comply with the limits for a Class B digital device, pursuant to part 15 of the FCC Rules. These limits are designed to provide reasonable protection against harmful interference in a residential installation. This equipment generates, uses and can radiate radio frequency energy and, if not installed and used in accordance with the instructions, may cause harmful interference to radio communications. However, there is no guarantee that interference will not occur in a particular installation. If this equipment does cause harmful interference to radio or television reception, which can be determined by turning the equipment off and on, the user is encouraged to try to correct the interference by one more of the following measures:

    Reorient or relocate the receiving antenna.
    Increase the separation between the equipment and receiver.
    Connect the equipment into an outlet on a circuit different from that to which the receiver is connected.
    Consult the dealer or an experienced radio/TV technician for help.

WARNING
Any changes or modifications not expressly approved by the manufacturer could void the user’s authority to operate the equipment.
Quick Start
Prerequisites

    DYNAMIXEL Power Supply (12V SMPS, or compatible 12v battery.)
        See Compatibility Table
    PC with Windows, Linux or MacOS.
    Serial converter to communicate between your PC and DYNAMIXEL (U2D2, OpenRB-150)
    DYNAMIXEL Control Software

WARNING:

    Some software may not support all OS options. Be sure to read the eManual page of any software you wish to use to ensure compatibility.

NOTE:

    The U2D2 is a small size USB to Serial communication converter that enables control and operation of DYNAMIXEL servos directly from a connected PC.
    The U2D2 Power Hub simplifies the process of connecting an external power source to your U2D2 to supply power to your DYNAMIXEL.

Compatible Software with DYNAMIXEL
DYNAMIXEL Wizard 2.0

DYNAMIXEL Wizard 2.0 a configuration tool designed to simplify the setup, configuration and management of DYNAMIXEL servos.

The following features are provided by DYNAMIXEL Wizard 2.0:

    DYNAMIXEL Firmware Update
    DYNAMIXEL Error Diagnosis
    DYNAMIXEL Configuration and Testing
    DYNAMIXEL Real-time Data Plotting
    Generate & Monitor DYNAMIXEL Packets

DYNAMIXEL SDK

DYNAMIXEL SDK is a software development kit that provides DYNAMIXEL control functions for a variety of popular programming languages.

Supported Programming Laguanges and Features:

    C, C++, C#, Python, Java, MATLAB, LabVIEW
    Windows, Mac, Linux.
    ROS
    Arduino

NOTE: You can also use more variety of software. For more information, see the following to check software provided by ROBOTIS.

    DYNAMIXEL to software Compatibility Table
    Controller to software Compatibility Table

Connector Information
Item 	TTL
Pinout 	1 GND
2 VDD
3 DATA
Diagram 	
Housing 	
JST EHR-03
PCB Header 	
JST B3B-EH-A
Crimp Terminal 	JST SEH-001T-P0.6
Wire Gauge for DYNAMIXEL 	21 AWG
Communication Circuit

To control the DYNAMIXEL actuators, the main controller needs to convert its UART signals to the half duplex type. The recommended circuit diagram for this is shown below.
TTL Communication

NOTE: Above circuit is designed for 5V or 5V tolerant MCU. Otherwise, use a Level Shifter to match the voltage of MCU.

Drawings
XL430-W250 (New)

    Download XL430_new(pdf).pdf
    Download XL430_new(dwg).dwg
    Download XL430_new(stp).stp

XL430-W250 (Old)

    Download XL430.pdf
    Download XL430.dwg
    Download XL430.stp

FR12-H101K

    Download fr12_h101_ref.dwg
    Download fr12_h101_ref.pdf
    Download fr12_h101.stp

FR12-H104K

    Download fr12-h104.dwg
    Download fr12-h104.pdf
    Download fr12-h104.stp

FR12-H103GM

    Download fr12-h103.dwg
    Download fr12-h103.pdf
    Download fr12-h103.stp

FR12-S101K

    Download fr12_s101_ref.dwg
    Download fr12_s101_ref.pdf
    Download fr12_s101.stp

FR12-S102K

    Download fr12_s102_ref.dwg
    Download fr12_s102_ref.pdf
    Download fr12_s102.stp

FR12-G101GM

NOTE: FR12-G101GM is a gripper frame set that include a FR12-E170 and FR12-E171.
FR12-E170

    Download fr12-e170.dwg
    Download fr12-e170.pdf
    Download fr12-e170.stp

FR12-E171

    Download fr12-e171.dwg
    Download fr12-e171.pdf
    Download fr12-e171.stp

Moment Of Inertia

    Download XL430,XC430 Moment of Inertia

© 2026 ROBOTIS. Powered by Jekyll & Minimal Mistakes.
