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
              - January 4th, We attempted the Track Width Tuning test phase, we got a max angular velocity of 2.9 radians or 166.67 degrees. We noticed that our robot on the FTC dashboard was not following the target nor moving at all. 
              - January 7th (caught up to the present), we figured out that the problem was that one of the wires for the parallel wheel was plugged into the wrong port. We simply changed the port in the code
                - The Turn Test Tuning was simple and worked well
                - Next, we tuned the Follower PID, Back and Forth Test, the results:
                  Heading_PID: kP = 6.5, kI = 0, kD = 0
                  Translational_PID: kP = 8, kI = 0, kD = 0.5
                - We accidently skipped the FollowerPIDTuner, which we will go back and tune during our next meeting
                - Moving onto the spline test, it worked very well moving forwards, but weirdly skips a part of the path when sliding backwards
                  - Need to retune localizer

Feb 4. 2023   - Successful Autonomous (A month has passed)
            
              - Let's begin with the completion of roadrunner tuning
                - We did the FollowerPIDTuner, which went well as it was pretty accurate
                - We checked the localizer again, which gave us near the same result as we initally had. The problem turned out to a lack of traction in one of the dead wheels so we tightened the spring on it
              - Building Autonomous (Track 1)
                - Our inital goal for autonmous was to simply score a cup
                - In our first track, we would push the signal cup our of our way and score a cup on the low goal
                - We learned to use RoadRunner's built in methods as well as how the FTC Cartesian Coordinate system worked
                - We focused specifically on the left side of the alliance side
              - January 14th (Competition / League meet)
                - We tried our best to finish up Track 1
                - Halfway through the competition, we could successfully score into the low goal at least 90% of the time
                - In the last match, we decided to try to mirror our left auto with our right to be compatible with our sister team, who happened to be our alliance partner
              - Afterwards
                - We hard our usual meetings and occasionally had extra meetings
                - There were a couple meetings where some wires were switched around, messing up the configuration 
                  - The most common switch that would occur were the dead wheels, to fix that, we would either rewire them or simply change it in the code
                  - To figure out if it was the dead wheels, we would just run the BackandForth autonomous to watch how the robot moved
                - During the competition, we realized that we wanted to create a circuit to maximize points, and to do that, we wanted change Autonomous
              - Redesigning Autonomous (Track 2)
                - We kept the idea of pushing the signal cup away and scoring in the low goal to complete part of the circuit
                - We added a section to grab a cup from the side stack and score a cup (or 2, maybe 3) in the high goal
                - Lastly, we wanted it to scan our custom signal sleeve and park in the correct spot
              - Optimizing Track 2 (There's a lot)
                - To ensure that we do not waste time closing the claw in the beginning, we simply closed it when we initialize
                - We wanted to save time so that we could possibly have time to score more cups than we planned by moving accessories (lift, extend, claw) while the robot was moving to different locations
                  - An example of this is when we only planned to score one low goal and one high goal, but saved time and ended up with enough time to score another high goal and park
                  - We also scan the cone while moving, the only margin of error being if the cup spins while it is being pushed
                - Another way we saved time was to eliminate or decreased certain wait times
                  - We decreased the time it took to wait to open and close the claw
                - We figured out a way to temporary increase the max velocity and acceleration to quickly move to places, with a little less accuracy
              - Issues and solutions with Track 2
                - Sometimes we needed an exact coordinate or angle to score a cup, which too a fair amount of time
                - We got the color sensor to work and tuned it well, however we had a little bit of touble learning to add it onto our roadrunner trajectory
                - We were having problems with the lift motors acting weird so we decided to just rotate one motor instead of both, which worked
                - When we were testing Track 2, we weren't testing it on the actual field so there were some setbacks
                - Telemetry would not work how we wanted it to, so we found alternatives by just testing the robot physically
              - We made the code look a lot cleaner than it originally was
