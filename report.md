## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

//Add gif with rviz vizualization

// Add image with DH paramenters derivation


Links   | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
------- | ---------- | ------ | ------ | --------
0 -> 1  | 0          | 0      | d1     | q1
1 -> 2  | -pi/2      | a1     | 0      | q2 -pi/2
2 -> 3  | 0          | a2     | 0      | q3
3 -> 4  | -pi/2      | a3     | d4     | q4
4 -> 5  |  pi/2      | 0      | 0      | q5
5 -> 6  | -pi/2      | 0      | 0      | q6
6 -> G  | 0          | 0      | dg     | 0


To convert an and dn parameters, it is necessary to check urdf file with location of joints.

Inspecting `kr210.urdf.xacro` file we can find the position for each joint and calculate the dimentions of the links
```
  <joint name="fixed_base_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
  <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
  <joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
  <joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
  <joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
  <joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
  <joint name="gripper_joint" type="fixed">
    <origin xyz="0.11 0 0" rpy="0 0 0"/><!--0.087-->
```

a1 =  0.35
a2 =  1.25
a3 = -0.054
d1 =  0.33 + 0.42
d4 =  0.96 + 0.54
dg =  0.193 + 0.11


The numerical DH parameters are listed bellow
Links   | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
------- | ---------- | ------ | ------ | --------
0 -> 1  |  0         |  0     | 0.75   | q1
1 -> 2  | -pi/2      |  0.35  | 0      | q2 -pi/2
2 -> 3  |  0         |  1.25  | 0      | q3
3 -> 4  | -pi/2      | -0.054 | 1.5    | q4
4 -> 5  |  pi/2      |  0     | 0      | q5
5 -> 6  | -pi/2      |  0     | 0      | q6
6 -> G  |  0         |  0     | 0.303  | 0



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]

