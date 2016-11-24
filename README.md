# Challenge 4: Autonomous Driving with Remote Monitoring and Control

### Task
Create a system that drives your vehicle through an indoor course without operator (human) control. The system should leverage control strategies for driving the collision avoidance (from challenge 4) and indoor positioning (from challenge 6). Your solution should also provide a real time data stream (challenge 3) including position information that should be displayed on a remote computer console (challenge 2) illustrating position information of your vehicle. Your system should also allow a human to take control remotely (challenge 3).

### You will integrate the following solutions
- [ ] Autonomous driving and collision avoidance using provided sensors, speed control and servo steering
- [ ] Indoor location sensing and reporting to a base station to log location information at the base station
- [ ] Real-time visualization of position based on logged information, displayed at a remote terminal
- [ ] Remote control from a disparate IP network to permit course corrections and safety features and to take over control of vehicle
- [ ] First-person video streaming from the crawler to the remote site where the crawler is being controlled

### Notes
- [ ] The course will use the single loop around the west end of the 4th floor in Photonics passing the windows at the West end and passing the elevators in the middle.
- [ ] The use of the pi as host for position data is a bonus
- [ ] The use of multiple access points is encouraged to overcome limits of a single Linksys router
- [ ] Please see the rubric for the required and qualitative assessments.

## System Components
- [ ] 4 Arduinos w/Xbees  : Indoor Localization 
- [ ] 1 Arduino w/XBee    : Vehicle control and sampled RSSI vector
- [ ] 1 RPI w/XBee        : Receive and stor data in SQL and host webpage ???
- [ ] Off-Network PC      : Demonstrate the system
- [ ] 1 Webcam            : FPV streaming and control from the UI

### To-Do
- [ ]
- [ ]

### Ideas
- If we can connect the XBee to a photon then we may have better luck transmitting the data in the space using multiple routers instead of one PAN direct line of communication network.
- We may also want to try and setup trip sensors using IR LEDs at each corner which would trigger the vehicle to turn using IR detectors on the vehicle 
