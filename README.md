# warmup_project


## Drive in Square 
I am using an approach with python's built in time module, specifically the sleep() function. Essentially, my robot travels forward for some set amount of time and then turns.  

My robot travels forward with an acceleration function "ramp_up", which steps the velocity in the x direction from 0 to the final speed (in this case 0.5) in 0.1 m/s intervals. I also wait 0.5 seconds before incrementing each speed to make the jumps less dramatic. I implemented this because the large jump between 0 and 0.5 m/s caused the robot to drift. The robot still drifts a little bit, but it is far less extreme than it was without it. 

Once the robot has reached its top speed, it continues at that speed for 5 sec. Then, it deccelerates with the "ramp_down" function, which decreases the robot's speed from 0.5 m/s to 0 (stopped) again in 0.1 m/s intervals. Here I wait 1 sec between each decrement, as if the robot is traveling too fast before it stops it tended to undershoot the turn. 

Then, I make the turn by setting the x velocity to 0 and setting the angular velocity to 0.2 (rad/sec). I need to turn 90 degrees, or /pi/2 radians, so if I am turning at 0.2 rad/sec, I need to wait for /pi seconds until I have reached a right angle. Then I set the angular velocity to 0 and the process repeats, and the robot keeps driving in a square.

I was unsure if the robot was supposed to drive in a square once, then stop, or if it was supposed to be continuous. My program at the moment does continuous squares. There is some drift that becomes more and more noticeable with each square the robot completes. Below is a gif of the square path the robot takes. Note that there is still some drifting, as the robot doesn't exactly end up at the origin.

![Driving in a square](DriveInSquare2.gif)



## Wall Follower
My wall follower program essentially tries to keep a wall at the robot's right side. The robot searches for the closest wall with its sensor, turns and travels towards it, and then rotates and travels forward while keeping the wall at its right. It also checks the sensor data directly in front of it to check when it is encountering a corner, and turns left at these occasions. 

The instructions for the robot are held in the FollowWall class. In "init", I subscribe to "/scan" which gives me the sensor data, and set up a publisher to "/cmd_vel" so that I can move the robot. I have a generic "change_speed" helper function which takes in a linear and angular velocity and applies this to the "cmd_vel" publisher. I use this helper function often in my "process_scan"  function which contains the bulk of the logic. I first find the closest wall to the robot as well as it's relative angle. I also keep track of the distance to a wall directly in front of the robot (if it exists). First, I check if this distance in front of the robot is very small, meaning the robot is about to crash, and if it is I stop and rotate the robot to the left. If not, I check if the robot is the appropriate distance from the wall, and if the closest wall is to the right. If it is aligned properly with the wall, I just move forward, and if not, I stop and rotate until it is aligned. If the robot is too far from the wall, I first rotate it so that it faces the wall and then drive it towards the wall. Finally, if the robot is too close to the wall, I rotate it to the left (again stopping if it is about to crash, and moving forward if it is not). In most of the angular adjustments I use proportional control where the error is how far off from the desired angle the robot is. When traveling around the room, the robot drives at a fixed speed. You can see this proportional control in action when the robot drives around the wall, from the red arrow swinging back and forth to try to align itself to be parallel to the wall.  

![Following wall](wallfollower.gif)

## Person Follower
My person follower code uses a very similar approach to my wall follower code. Essentially the robot uses its sensor to locate the object closest to it. Then it rotates towards the object, and once it is roughly facing it, it moves forward and stops a fixed distance away. 

I describe these behaviors in my FollowPerson class. Again, in "init", I subscribe to "/scan" which gives me the sensor data, and set up a publisher to "/cmd_vel" so that I can move the robot. I again have a generic "change_velocity" helper function which takes in a linear and angular velocity and applies this to the "cmd_vel" publisher. My "process_scan" function first finds the closest object to the robot and its relative angle. If a minimum distance can't be found, the robot doesn't move. If the minimum distance is greater than the distance I want my robot to be from the object, I frist rotate the robot so that it is roughly facing the object and then move it forward. I again use proportional control for both the linear and angular components of the robot's velocity, which are proportional to the distance the robot is from the object and the angular offset, respectively. Finally, once the robot is the appropriate distance away, it stops. I try to run through each of these conditions in the gif below. 

![Following person (cylinder)](personfollower.gif)

## Challenges
By far the behavior that gave me the most problems was the wall follower. In particular I ran into a lot of issues with the robot getting stuck in the corners. 

## Future Work

## Takeaways
* This kind of work involves a ton of trial and error.

