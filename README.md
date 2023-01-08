# Bazinga-2022-2023

Before Nov 12, 2022  - Had a lot of problems / Focused on building the robot first

              - Teleop would not load onto the Control Hub
                - Fixed by upgrading Gradle
              - Had to run with LinearOpmode
              - Object nullpoint error
                - Fixed this by deleting LinearOpmode and using Opmode
                
Nov 12, 2022  - Teleop with Opmode (not LinearOpmode) test successful

              - Imported temporary autonomous into folder
              - Almost ready to try to program RoadRunner

Jan 7, 2023   - It's been quite some time, so this is going to be pretty long

              Let's start where we ended:
              - While the programmers waited on the construction of the robot, we created a basic Teleop with functions to all of the known mechanisms 
              - Additionally, we continued our research on RoadRunner and began setting it up
                - We configured the DriveConstants, SampleMecanumDrive, StandardTrackingWheelLocalizer, and TwoWheelTrackingLocalizer to the best of our abilities without the robot
              - We decided that instead of using a camera in autonomous, we were going to use a color sensor (something which we have never used before)
                - We found the documentation of the color sensor v3 on Rev Robotics' website
                - We tried it out using telemetry to see how accurate it was in a certain amount of distance
                - We adjusted values until we got numbers to our liking that were easily understandable
                - The color sensor is attached to the back of our robot and will back up to the cone in order to see it
              - December 3rd, the robot was fully built including two dead wheels, a mecanum drive, and a dual lift and claw mechanism
              - December 7th, we attached a second color sensor onto the claw so that when it senses an object, it automatically closes
              - December 10th, we began tuning roadrunner
                - The first test was the localization test, but we ran into a couple hardware issues such as the parallel dead wheel not enough tension and not touching the ground, causing the data to be incorrect and inconsistent
              - December 14th, since we were only using dead wheels without drive encoders, we used the feedforward tuning method the final results of that ended up being: 
                kV = 0.0265 
                kA = 0
                kStatic = 0.133
                
                Afterwords, we ran the straight test with a 1% error within our specified distance.
                - When we ran the strafe test, there was a very noticable forward drift. We discovered that we reversed the wrong wheels, which impacted our driving teleop. We switched around some of the +/- signs to fix that problem. The drift however, was still present; so it was concluded to be a mechanical error in the motors.
              - 
                
