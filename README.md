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

![Ain't it pretty!][image4] 

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

This is a lot of code and very boring, but I was told needed to be here. 

First, here is our above DH table in code form:

```
s = {alpha0:      0, a0:       0, d1:  0.75,
     alpha1:  -np.pi/2, a1:    0.35, d2:     0, q2: q2-np.pi/2,
     alpha2:      0, a2:    1.25, d3:     0,
     alpha3:  -np.pi/2, a3:  -0.054, d4:  1.50,
     alpha4:   np.pi/2, a4:       0, d5:     0,
     alpha5:  -np.pi/2, a5:       0, d6:     0,
     alpha6:      0, a6:       0, d7: 0.303, q7: 0}
```

I should point out that q1 through q6 are the rotation variables and do not appear above, but that is ok.

This first section is creating the transform matrices about each point:

```
    T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
    T5_6 = T5_6.subs(s)

    T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
    T6_7 = T6_7.subs(s)
```

So, if we pick one as an example, T4_5, represents going from Joint 4 to 5.  Get it? Easy!

Now, to combine them, it is just simple multiplication, for example:

```
T0_7 = ((((((T0_1 * T1_2) * T2_3) * T3_4) * T4_5) * T5_6) * T6_7)
```

Means that T0_7 is now the transform from base_link to gripper_link.  Well, almost.  The URDF file that describes the arm does two final
twists to the gripper that our table doesn't include.  So we have to account for them:

```
 R_zz = Matrix([[ cos(np.pi), -sin(np.pi), 0,0],
              [ sin(np.pi),  cos(np.pi), 0,0],
              [          0,           0, 1,0],
              [          0,           0, 0,1]])
    R_yy = Matrix([[  cos(-np.pi/2), 0, sin(-np.pi/2),0],
              [              0, 1,             0,0],
              [ -sin(-np.pi/2), 0, cos(-np.pi/2),0],
              [              0, 0,             0,1]])
    R_corr = R_zz * R_yy
```

And then apply them to get our final base to gripper transform:

```
 T_total = (T0_7 * R_corr)
```

So that T_total is the generic one with symbols instead of actual values.  This was just mostly used as a forward kinematics check,
because this rely on giving values for the symbols of the angles, which is exactly what we need to figure out!

So, we need to come up with the transform using the end effector's position and Roll, Pitch, Yaw.

To accomplish this, I set up the Roll, Pitch, Yaw transforms like this:

```
R_z = Matrix([[ cos(yawsym), -sin(yawsym), 0],
              [ sin(yawsym),  cos(yawsym), 0],
              [          0,           0,   1]])

R_y = Matrix([[  cos(pitchsym), 0, sin(pitchsym)],
              [              0, 1,             0],
              [ -sin(pitchsym), 0, cos(pitchsym)]])
              

R_x = Matrix([[  1,            0,             0],
              [  0, cos(rollsym), -sin(rollsym)],
              [  0, sin(rollsym),  cos(rollsym)]])
```

And then substituted in the ones given from the End Effector (after converting from quaternions):

```
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
     [req.poses[x].orientation.x, req.poses[x].orientation.y,
      req.poses[x].orientation.z, req.poses[x].orientation.w])
Ree = (R_x * R_y * R_z).evalf(subs={rollsym:roll,pitchsym:pitch,yawsym:yaw})
```

This gives us the Rotational part of the final solution.  The last position side is just the given x,y,z.

So now we have a start, time to solve the problem.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

So the problem breaks down into finding the wrist, then the first three angles, then the last three angles.

##### Finding the wrist

The robot we are given is able to be decoupled and it took me a while to realize what that actually meant.  I made this video 
to explain:

[![How it works](http://img.youtube.com/vi/iDEIZv8zL9A/0.jpg)](http://www.youtube.com/watch?v=iDEIZv8zL9A "Finding the Wrist - TieDyedGuy")

So, I set the wrist to be joint 5 and joint 5 is .303 meters away from the end effector's position.  I needed to take the rotation
from the end effector and apply it to the end effector's position.  Here is that code:

```
wx = (px - (d6 + d7) * Ree[0,0]).subs(s)
wy = (py - (d6 + d7) * Ree[1,0]).subs(s)
wz = (pz - (d6 + d7) * Ree[2,0]).subs(s)
```

P. = End Effector's position, W. = Wrist positions.  The interesting thing was this uses the X portion of the rotation.  This was
because of those URDF rotations above.

Now that we have our wrist, we can decouple and figure out the Inverse Position Kinematics part:

##### The Inverse Position Kinematics: Theta 1 2 3



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

##### Finding the last three angles:

No way around it, this one you know your target, it is just brute force.  There is the TF.tranforms.euler_from_matrix() function that takes in the rotation matrix and what kind of axies you want back.  I set up a giant for loop through all the combinations to find the one that worked.  There is probably a trig way of solving it, but, that ain't me.

##### Overall:

This project was a challenge to me because of the amount of trig needed.  I feel as though I should be able to do the trig and will be refreshing myself more and more as time goes on.  I am glad this project is done.
