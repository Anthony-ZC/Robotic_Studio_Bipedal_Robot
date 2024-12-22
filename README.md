# Robot Studio (2024 Fall) Full Project Implementation  
It is a bipedal robot implementation for Columbia MECE 4611 Robotics Studio created by ***Zewen "Anthony" Chen*** and ***Xinhe "Carl" Yang*** (Fall 2024). We get inspiration by Yuhang Hu's work [CB-20](https://www.youtube.com/watch?v=Y0fBdpLf9ZI&t=1s) and redesign it.

For those who attend Robotics Studio in the future or want to build their own first bipedal robot, I hope this implementation can help you. But remember, it is far from perfect. <u>**So, do not just copy it, make it better!**</u>  

Best wishes,  
Anthony Chen  

---

### Project Overview

- **Project Video**  
  - [x] Playlist Link: [https://www.youtube.com/playlist?list=PLH_mFWdIEgZGXP19iW7NsymILhA3eG9Pi](https://www.youtube.com/playlist?list=PLH_mFWdIEgZGXP19iW7NsymILhA3eG9Pi)  
  - [x] Journey Video Link: [https://www.youtube.com/watch?v=UUYPjQSJz8M](https://www.youtube.com/watch?v=UUYPjQSJz8M)  

- **CAD**  
  - [x] CAD Link (GrabCAD): [https://grabcad.com/library/robotics-studio-bipedel-robot-1](https://grabcad.com/library/robotics-studio-bipedal-robot-1)  

- **3D Printing**  
  - [x] [STL Files](./3D%20Printing/)  
  - [x] [3MF File](./3D%20Printing/Robot.3mf)  
  - [x] MarkerWorld Link: [https://makerworld.com/zh/models/903789#profileId-863432](https://makerworld.com/zh/models/903789#profileId-863432)  

- **Assembly Guidance**  
  - [x] [Essential Parts Links](#essential-parts-links)  
  - [ ] Assembly Tips
  - [ ] Assembly Video in CAD (not sure to do...)

- [**Control Code**](#control-code)
  - [x] [Requirements](#requirements)
  - [x] Testing Code
  - [x] Integrated Control Code
  - [ ] Servo Curve Analysis

- **Simulation Code**
  - [ ] STL
  - [ ] URDF
  - [ ] Physical Properties
  - [ ] Simulation Code
---

### Essential Parts Links  

Here is a directory of purchase links for essential parts:  

- **Amazon**  
  - [LX-16A Serial Bus Servo](https://www.amazon.com/Hiwonder-LX-16A-Robotic-Controller-Control/dp/B073XY5NT1)  Quantity required: <u>**7**</u> 
  - [Additional LX-16A Serial Bus Servo](https://www.amazon.com/LewanSoul-Real-Time-Feedback-Bearing-Brackets/dp/B0748BQ49M) 
  - [Battery Pack](https://www.amazon.com/dp/B01M7Z9Z1N)  Quantity required: <u>**1**</u> (or any other battery have DC 12V/5V USB Dual Output) 
  <u>**Warning</u>: This battery may not afford more than 8 servos**
  - [DC to DC Converter](https://www.amazon.com/gp/product/B00BYS9JYA) Quantity required: <u>**1**</u>
  - [Raspberry Pi 4](https://www.amazon.com/Raspberry-Model-2019-Quad-Bluetooth/dp/B07TD42S27?th=1) Quantity required: <u>**1**</u>
  - [Micro SD Card (For Raspberry Pi system)](https://www.amazon.com/dp/B073JWXGNT?th=1) Quantity required: <u>**1**</u>
  - [USB Male to USB C](https://www.amazon.com/Braided-Compatible-Samsung-Portable-Charger/dp/B07HQHL6ZK?th=1) Quantity required: <u>**1**</u>
  - [Raspberry Pi Touch Screen](https://www.amazon.com/dp/B0D4DL38MF?ref=ppx_yo2ov_dt_b_fed_asin_title) Quantity required: <u>**1**</u> 
 <u>**Warning</u>: Please follow the driver installation guidance in the product to enable it on Raspberry Pi. You may not be able to use  <u>**HDMI</u> output if you install the driver. If it does not work, please delete "LCD- show" floder and install it again.**
  - [USB 2.0 Mini Microphone](https://www.amazon.com/dp/B071WH7FC6?ref=ppx_yo2ov_dt_b_fed_asin_title) Quantity required: <u>**1**</u>
  - [M2 Male Female Hex Brass Standoffs](https://www.amazon.com/dp/B06XCNF6HK?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)  Quantity required: <u>**1 pkg**</u>

- **AliExpress**  
  - [3x16xM2 Shoulder Screw](https://www.aliexpress.us/item/3256806009271581.html?spm=a2g0o.order_list.order_list_main.10.12f01802Rwg7LO&gatewayAdapt=glo2usa) Quantity required: <u>**8**</u>


- **McMaster**  
  - [Screws Set - Example Link](https://mcmaster.com/example1)  
  - [Bearings Set - Example Link](https://mcmaster.com/example2)  

---

### Control Code  
#### Requirements
 - Python 3.10.15+ (To install PyLX-16A package, you must use a Python environment with 3.10 or newer version.)
 - [PyLX-16A](https://github.com/ethanlipson/PyLX-16A/tree/master) 
 - Numpy 2.1.3+
 - pyserial 3.5+
 - PyQt6 6.7.1+ (for visual debugging on windows, **cannot process on RasoberryPi**)
 - speech_recognition (for voice recognition)
 - Pygame (for screen facial expression)
#### Testing Code
 - servo-test.py
    Visualized servo adjusting code.
 - test_hello-world.py
    3 servos sinusoidal testing code.

---
