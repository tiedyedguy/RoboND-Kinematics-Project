## Project: Kinematics Pick & Place
[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[image4]: ./misc_images/DHDiagram.PNG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

So, when I started, I tried to write up some DH parameters and well, I failed.  So, the video kinda shows us the answer, so I went with that and figured it would be fine.  It was and I'm done.  Now though, I've gone back and looked and doing the project helped me understand the DH parameters that I feel I could do them for the next robot.

Anyway, here they are!

Alpha | A | D | Theta
--- | --- | --- | ---
0| 0| 0.75 | Q1
PI/2| 0.35| 0| Q2- PI/2
0| 1.25|0|Q3
-PI/2|-0.054| 1.50|Q4
PI/2| 0|0|Q5
PI/2|  0|   0|Q6
0|   0| 0.303|0

So the picture that represents this is:

![Ain't it pretty!][image_4] 

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

This is a lot of code and very boring.  So I'll show you one segement that you just do a bunch more times:

```
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)
```


Lastly, to create the homogeneous transform you have the right size because that is the just x,y,z position you get.

To get the rotational, I got the Roll, Pitch, Yaw from the TF library, then created three rotational transforms for X_axis, Y_axis, and Z_axis.  Multiplied those together and put in the RPY.  


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

This part took me a while to understand how that works.  When I finally figured it out, I made this video to help my classmates:

[![How it works](http://img.youtube.com/vi/iDEIZv8zL9A/0.jpg)](http://www.youtube.com/watch?v=iDEIZv8zL9A "Finding the Wrist - TieDyedGuy")


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


First, I like to lead with results:

[![Watch it work](http://img.youtube.com/vi/jHT14vVb9vY/0.jpg)](http://www.youtube.com/watch?v=jHT14vVb9vY "Project 2 - TieDyedGuy")

So, let's talk about the main problems:  Find the wrist, find the first three angles, find the last three angles.

##### Finding the wrist: 

I would say that 80% of my answer was figured out based on what others were talking about.  The last 20% was random trial and error.  Everytime you think you have it, the numbers are off just slightly enough that you are left sad and confused.  Luckily, the lecture gets you somewhat there and then it isn't bad when you finally get that answer.

##### Finding the first three angles:

The first angle is so simple it is laughable.  Got that one right away.

The second two are a nightmare for a trig hater.  There is no brute forcing here.  I have about 15 pages of notes in front of me trying to figure out what is going on.  Everyone says "Law of cosines" and every google topic on RRR says to use it also.  I tried so many ways that I found, and none seem to work.  The last method that finally got me the answers was from a clue someone posted to the group.  Apparently my big problem was the 0.054 drop of the arm between 3 and 4.  Once they helped me with the trig, I was set.

##### Finding the first three angles:

No way around it, this one you know your target, it is just brute force.  There is the TF.tranforms.euler_from_matrix() function that takes in the rotation matrix and what kind of axies you want back.  I set up a giant for loop through all the combinations to find the one that worked.  There is probably a trig way of solving it, but, that ain't me.

##### Overall:

This project was a challenge to me because of the amount of trig needed.  I feel as though I should be able to do the trig and will be refreshing myself more and more as time goes on.  I am glad this project is done.
