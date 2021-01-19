# warmup_project


## DRIVE IN SQUARE: 
I am using an approach with python's built in time module, specifically the sleep() function. Essentially, my robot travels forward for some set amount of time and then turns.  

My robot travels forward with an acceleration function "ramp_up", which steps the velocity in the x direction from 0 to the final speed (in this case 0.5) in 0.1 m/s intervals. I also wait 0.5 seconds before incrementing each speed to make the jumps less dramatic. I implemented this because the large jump between 0 and 0.5 m/s caused the robot to drift. The robot still drifts a little bit, but it is far less extreme than it was without it. 

Once the robot has reached its top speed, it continues at that speed for 5 sec. Then, it deccelerates with the "ramp_down" function, which decreases the robot's speed from 0.5 m/s to 0 (stopped) again in 0.1 m/s intervals. Here I wait 1 sec between each decrement, as if the robot is traveling too fast before it stops it tended to undershoot the turn. 

Then, I make the turn by setting the x velocity to 0 and setting the angular velocity to 0.2 (rad/sec). I need to turn 90 degrees, or /pi/2 radians, so if I am turning at 0.2 rad/sec, I need to wait for /pi seconds until I have reached a right angle. Then I set the angular velocity to 0 and the process repeats, and the robot keeps driving in a square.

I was unsure if the robot was supposed to drive in a square once, then stop, or if it was supposed to be continuous. My program at the moment does continuous squares. There is some drift that becomes more and more noticeable with each square the robot completes. 


Below is a gif of the square path: 

![Driving in a square](DriveInSquare2.gif)
