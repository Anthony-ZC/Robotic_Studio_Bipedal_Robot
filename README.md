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
  - [x] [Assembly Tips](#assembly-tips)
  - [ ] Assembly Video in CAD  
  Probably won't do it anytime soon, I think a good robotics engineer should be capable of analyzing and assembling this robot.(Or just because I am lazy.) But if enough requests are posted in issues, perhaps I'll make it.

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
### Assembly Guidance
#### Essential Parts Links  

Here is a directory of purchase links for essential parts:  

- **Amazon**  
  - <u>**7**</u> [LX-16A Serial Bus Servo](https://www.amazon.com/Hiwonder-LX-16A-Robotic-Controller-Control/dp/B073XY5NT1) 
  - [Additional LX-16A Serial Bus Servo](https://www.amazon.com/LewanSoul-Real-Time-Feedback-Bearing-Brackets/dp/B0748BQ49M) (for those who do not take the lecture)
  - <u>**1**</u> [Battery Pack](https://www.amazon.com/dp/B01M7Z9Z1N) (You may use any other battery have DC 12V/5V USB dual output.)  
  <u>**Warning</u>: This battery may not be able to afford more than 8 servos.**
  - <u>**1**</u> [DC to DC Converter](https://www.amazon.com/gp/product/B00BYS9JYA)
  - <u>**1**</u> [Raspberry Pi 4](https://www.amazon.com/Raspberry-Model-2019-Quad-Bluetooth/dp/B07TD42S27?th=1)
  - <u>**1**</u> [Micro SD Card (For Raspberry Pi system)](https://www.amazon.com/dp/B073JWXGNT?th=1)
  - <u>**1**</u> [USB Male to USB C](https://www.amazon.com/Braided-Compatible-Samsung-Portable-Charger/dp/B07HQHL6ZK?th=1)
  - <u>**1**</u> [Raspberry Pi Touch Screen](https://www.amazon.com/dp/B0D4DL38MF?ref=ppx_yo2ov_dt_b_fed_asin_title)  
 <u>**Warning</u>: Please follow the driver installation guidance in the product description to enable it on Raspberry Pi. You may not be able to use  <u>HDMI</u> output if you enable the touch screen. If the touch screen does not work, please delete "LCD- show" floder and install it again.**
  - <u>**1**</u> [USB 2.0 Mini Microphone](https://www.amazon.com/dp/B071WH7FC6?ref=ppx_yo2ov_dt_b_fed_asin_title)
  - <u>**1 pkg**</u> [M2 M3  Male Female Hex Brass Standoffs Spacers Screws Nuts Kit](https://www.amazon.com/dp/B06XCNF6HK?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1) (We only use M2 here.)
  - <u>**1 pkg**</u> [Raspberry Pi 4 Heatsink](https://www.amazon.com/dp/B0963BMGFY?ref=ppx_yo2ov_dt_b_fed_asin_title) (The 4mm-thick heatsink in it is perfectly adapted to the touchscreen.)
  - <u>**1 pkg**</u> [M3x6mmx0.5mm Stainless Steel Round Flat Washer](https://www.amazon.com/dp/B015A39NCC?ref=ppx_yo2ov_dt_b_fed_asin_title) (You may use any other 3mm flat washer with aorund 0.5mm thicknesses.)

- **AliExpress**  
  - <u>**8**</u> [3x16xM2 Shoulder Screw ](https://www.aliexpress.us/item/3256806009271581.html?spm=a2g0o.order_list.order_list_main.10.12f01802Rwg7LO&gatewayAdapt=glo2usa)


- **McMaster**  
  - <u>**1 pkg**</u> [M2 x 0.4 mm Thread, 8 mm Long, Stainless Steel Socket Head Screw](https://www.mcmaster.com/91292A832/)  
  - <u>**1 pkg**</u> [M2 x 0.4 mm Thread, 12 mm Long, Stainless Steel Socket Head Screw](https://www.mcmaster.com/91292A834/)  
  - <u>**2 pkg**</u> [for Plastic, 18-8 Stainless Steel, M2.2 Screw Size, 8 mm Long, Phillips Rounded Head Thread-Forming Screws](https://www.mcmaster.com/99461A922/)  
  - <u>**1 pkg**</u> [for M2 Screw Size, Standard, 2.4 mm ID, 4.4 mm OD, 18-8 Stainless Steel Split Lock Washer](https://www.mcmaster.com/92148A050/)  
  - <u>**1 pkg**</u> [Medium-Strength, Class 8, M2 x 0.4 mm Thread, Zinc-Plated Steel Hex Nut](https://www.mcmaster.com/90591A265/)  
  - <u>**1 pkg**</u> [M2 x 0.4 mm Thread Size, 2.9 mm Installed Length, Brass Tapered Heat-Set Inserts for Plastic](https://www.mcmaster.com/94180A307/)  
  - <u>**4**</u> [3 mm Shoulder Diameter, 25 mm Shoulder Length, M2 x 0.4 mm Thread Alloy Steel Shoulder Screws](https://www.mcmaster.com/92981A775/)  
#### Assembly Tips 
  - For 3D-printed "Robot_Cover" part, you need to do **thermal insert** through the soldering iron.
  - For "Cylinder_Pi" and "Cylinder_DC_Converter" parts in CAD, **do not 3D print it**, use [M2 studs](https://www.amazon.com/dp/B06XCNF6HK?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1) above instead. We have tried 3D print studs with thermal inserts but failed, that's why they are not included in 3D Printing files.
  - In Solidworks CAD, you may see **Internal-Tooth Lock Washer** for mating convenience in Solidworks. However, you can use other less damaging lock washers like [Split Lock Washer](https://www.mcmaster.com/92148A050/) instead.
  - For "Raspberry_Pi_Platform" part, you can use **laser cut acrylic sheets** to get it instead of 3D printing
  - Although the screws that connect the **"Raspberry Pi"**, the **"Raspberry_Pi_Platform"**, the **"Servo_Connector_Head""**, and the **"studs that hold the DC converter"** in place in CAD do not use a **lock washer,** it is highly recommended that you use one in order to prevent parts from falling out of place during the robot's motion.
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
