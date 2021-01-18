# warmup_project


DRIVE IN SQUARE: 
I am using an approach with python's built in time module, specifically the sleep() function. The robot drives forward for 5.0 seconds, since I set the velocity to 0.1 in the x direction and then put the program to sleep for 5 seconds. Then I make the turn by setting the x velocity to 0 and setting the angular velocity to 0.2 (rad/sec). I need to turn 90 degrees, or /pi/2 radians, so if I am turning at 0.2 rad/sec, I need to wait for /pi seconds until I have reached a right angle. Then I set the angular velocity to 0 and the process repeats, and the robot keeps driving in a square.

I was unsure if the robot was supposed to drive in a square once, then stop, or if it was supposed to be continuous. My program at the moment does continuous squares. 
