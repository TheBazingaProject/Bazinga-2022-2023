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
              - *December 3rd*, the robot was fully built including two dead wheels, a mecanum drive, and a dual lift and claw mechanism
              - We attached a second color sensor onto the claw so that when it senses an object, it automatically closes
