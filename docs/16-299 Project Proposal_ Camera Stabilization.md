 # **Controls Project: Analysis of Camera Stabilization Control Methods**

Note: I promise the only reason this doc is 4 pages is because of formatting T-T it should still be easy to read and understand…

# **Table of Contents** {#table-of-contents}

[Table of Contents	1](#table-of-contents)

[I. Goals	1](#goals)

[II. Why?	1](#why?)

[III. Process	1](#process)

[Part A: 2-Axis Mechanical Gimbal Hardware Development	1](#part-a:-2-axis-mechanical-gimbal-hardware-development)

[Part B: Frequency-Domain PID Controller	2](#part-b:-frequency-domain-pid-controller)

[Part C: Alternative Control Methods— Linear-Quadratic Regulator	2](#part-c:-alternative-control-methods—-linear-quadratic-regulator)

[Part D: Alternative Control Methods— Fuzzy-PID	2](#part-d:-alternative-control-methods—-fuzzy-pid)

[Part E: Analysis and Comparison of Methods	2](#part-e:-analysis-and-comparison-of-methods)

[Current Unknowns	2](#current-unknowns)

[IV. Bill of Materials	3](#bill-of-materials)

[V. Extensions	3](#extensions)

[VI. References	3](#references)

1. # **Goals** {#goals}

* Design and implement a simple custom handheld camera device gimbal  
* Achieve \[some statistic\] image stabilization via with a PID controller method  
* Test and analyze various controller methods (PID, LQR, Fuzzy-PID) for efficient and accurate image stabilization

2. # **Why?** {#why?}

* idk, cuz it’s cool

3. # **Process** {#process}

## Part A: 2-Axis Mechanical Gimbal Hardware Development {#part-a:-2-axis-mechanical-gimbal-hardware-development}

*Subgoal: develop a basic, 2-axis handheld gimbal setup.*   
*Timeline: 2 weeks*  
	All that is necessary for this section is to design (minimally, for I am not a Mech-E) and build a simple handheld gimbal, something akin to those in references \[4\] and \[5\]. See the full [bill of materials](#bill-of-materials) (BOM) for what I intend to use, but the main items are brushless DC motors (three as a potential extension), an IMU with both gyro and accelerometer, a camera, and a microcontroller to interface with these devices.

## Part B: Frequency-Domain PID Controller {#part-b:-frequency-domain-pid-controller}

*Subgoal: Program one controller type for a 2-axis mechanical gimbal.*  
*Timeline: 2 weeks*  
	It will be a later goal to try other controllers, but first I need to get the gimbal working with a PID control system just to know that it works. I will need to run simulations to find the right gains, for which I will use Matlab.   
	One part of this phase will simply be developing the codebase to drive the gimbal system at all, and the other part will be implementing a controller interface and using Matlab to simulate the setup so that I have actual numbers to plug in for the physical system.

## Part C: Alternative Control Methods— Linear-Quadratic Regulator {#part-c:-alternative-control-methods—-linear-quadratic-regulator}

*Subgoal: Program my existing controller interface with the LQR state-space method.*  
*Timeline: 1 week*  
	As described; re-implement my controller interface with the LQR algorithm.

## Part D: Alternative Control Methods— Fuzzy-PID {#part-d:-alternative-control-methods—-fuzzy-pid}

*Subgoal: Program my existing controller interface with the LQR state-space method.*  
*Timeline: 1 week*  
As described; re-implement my controller interface with a modified PID \+ fuzzy-logic controller.

## Part E: Analysis and Comparison of Methods {#part-e:-analysis-and-comparison-of-methods}

*Subgoal: Program my existing controller interface with the LQR state-space method.*  
*Timeline: 1 week*  
	For this section, I’ll need to track and plot various statistics from each implementation and then do comparisons on them. Things I might track:

* Error plot vs. time  
* Motor torques vs. time  
* Motor velocities vs. time

With these— and maybe other tracked values as I come across them— I hope I can build a picture of how each algorithm behaves in comparison to one another.

## Current Unknowns {#current-unknowns}

*What should be my expected stabilization threshold?*  
	I stated in my ‘Goals’ section that I would like to meet a certain standard for image stabilization, but what should that be? And how will I measure that? One idea is to track how much in-frame object movement I can eliminate based on pixel location, but I’m not sure I have the time to implement the math to do so. (Or the hardware– my arduino might be stretched thin as-is.) I might drop the metric threshold and just eyeball how nice my stabilization looks.  
*How will I measure ‘efficiency’ and ‘accuracy’ across different controller methods?*  
	Similar to the last point, but I think I can use the metrics I already intend to track for analysis purposes to determine these qualities. I.e., accuracy could be based on overall error or the rate of change of error per time; efficiency could be based on maximum torque or velocity.

4. # **Bill of Materials** {#bill-of-materials}

| Qty. | Item | Price | Function |
| :---- | :---- | :---- | :---- |
| 2 | [Gimbal Motor with Encoder](https://www.sparkfun.com/gimbal-motor-with-encoder-12v-587rpm.html) | $62/ea | Gimbal actuators |
| ~~1~~ | ~~[Runcam 6 Action Camera](https://shop.runcam.com/runcam-6/)~~ | ~~$99~~ | ~~Gimbal test camera— I picked this because I can interface with it using [USB-C or maybe UART](https://store-m8o52p.mybigcommerce.com/product_images/img_RunCam6/RunCam%206%20_manual_en.pdf). Image stabilization can be disabled.~~ |
| ~~5~~ | ~~[A23 12V batteries](https://www.digikey.com/en/products/detail/energizer-battery-company/A23BPZ/5431480)~~ | ~~\~$3~~ | ~~Motor power supply, multiple b/c I’m not sure how many I’ll need~~ |
| 2 | [9V batteries](https://www.digikey.com/en/products/detail/duracell-industrial-operations-inc/PC1604/16344168) | \~$3 | CPU power supply, one extra for backup |
| ~~1~~ | ~~[A23 battery clip](https://www.digikey.com/en/products/detail/mpd-memory-protection-devices/BH23AW/2439315)~~ | ~~$2~~ | ~~To hold the 12V batteries~~ |
| 1 | 9V battery clip w/ power plug | $0 | To hold the 9V batteries (already owned) |
| 1 | Arduino Uno R3 (or Elegoo Mega 2560?) | $0 | Central processing unit (already owned) |
| 1 | L298N Motor Controller | $0 | Motor driver (already owned) |
| 1 | MPU6050/GY-521  | $0 | Gimbal IMU sensor (already owned) |

*Note: any items listed above for $0 are items which I already own and do not need to source.*

5. # **Extensions** {#extensions}

*3-Axis Gimbal \+ Controller Analysis*

* Essentially, perform the same experiment as described above, but using a 3-axis gimbal instead of 2\.   
* What I would’ve done, if time/my sleep schedule allowed for it.

*Electronic Image Stabilization*

* Explore electronic image stabilization techniques, with- and without gyro sensor data as input.  
* Concurrently run mechanical and digital image stabilization for greater smoothness of video.  
* Also part of my original idea, but too ambitious for now.

*Custom Frame and Hardware Design*

* Design a custom PCB and frame, likely 3D printed or in some part machined.  
* Get comfortable with custom CAD design and manufacturing.  
* Pretty out-of-scope for this class, but aligns well with my embedded systems coursework and other interests.

*Other Control Methods*

* Consider implementing other methods I encountered: model-based or model-free adaptive control, sliding mode control, H-infinity.  
* All too complicated to learn in this timeframe.

6. # **References** {#references}

1. [Testing and comparison of different control methods on a gimbal system](https://dergipark.org.tr/en/download/article-file/3473785)   
2. [Practical implementation of a PID-Model Reference Adaptive Controller for a 2-DOF Camera Gimbal](https://www.academia.edu/112270158/Practical_implementation_of_a_PID_Model_Reference_Adaptive_Controller_for_a_2_DOF_Camera_Gimbal)   
3. [CAMERA GIMBAL STABILIZATION USING CONVENTIONAL PID CONTROLLER AND EVOLUTIONARY ALGORITHMS](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7375580)  
4. [DIY Gimbal | Arduino and MPU6050 Tutorial](https://www.youtube.com/watch?v=UxABxSADZ6U)  
5. [DIY Gimbal for you Action Camera | Full Build Video](https://www.youtube.com/watch?v=ePBoH62zgzc)  
6. [1 axis gimbal](https://www.youtube.com/shorts/0_cIXT7tir0)  
7. [DPReview TV: Why electronic image stabilization works better on your GoPro than your camera](https://www.youtube.com/watch?v=1IF-k0_GWjg)  
8. [Image Stabilization in Videography | ProGrade Digital](https://progradedigital.com/understanding-and-using-image-stabilization-in-videography/#:~:text=Optical%20Image%20Stabilization%20\(OIS\)%20is,high%2Dfidelity%2C%20sharp%20images)  
9. [Homemade 2-Axis DSLR Brushless Gimbal Test](https://www.youtube.com/watch?v=_dG2DLSAPJE)  
   1. [https://danielrhyoo.com/subpages/projects/brushlessGimbal.html](https://danielrhyoo.com/subpages/projects/brushlessGimbal.html)  
10. [DIY Self Balancing Gyroscopic Camera Stabilizer](https://www.youtube.com/watch?v=cjV-yDNdeOI)  
11. [Deepseek Conversation: Project brainstorming](https://chat.deepseek.com/share/d1ming45k85dxv20gq)  
12. [GitHub \- wagiminator/ATtiny13-TinyUPS: Uninterruptible Power Supply](https://github.com/wagiminator/ATtiny13-TinyUPS)   
13. [In-Depth: Interface L298N DC Motor Driver Module with Arduino](https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/)   
14. [Modeling and Simulation of Two Axes Gimbal Using Fuzzy Control](https://www.techscience.com/cmc/v72n1/46831/html#fig-2)   
15. [Modeling and Control of a Two-Axis Stabilized Gimbal Based on Kane Method](https://www.mdpi.com/1424-8220/24/11/3615)   
    1. [Dynamics: Introduction to Kane's Method](https://nescacademy.nasa.gov/downloadFile/1324)   
16. [Satellite Mission Analysis \- MATLAB & Simulink](https://www.mathworks.com/help/aerotbx/satellite-mission-analysis.html?s_tid=CRUX_lftnav) 