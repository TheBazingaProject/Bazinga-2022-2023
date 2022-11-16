/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This particular OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop Tank", group="Robot")

public class TeleopChosenOne extends OpMode{

    /* Declare OpMode members. */
    ElapsedTime runtime = new ElapsedTime();
    Hardwaremap robot = new Hardwaremap();
    boolean reverse = false;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Send telemetry message to signify robot waiting;
        robot.init(hardwareMap);
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.bleft.setPower(1.0);
        double ly = gamepad1.left_stick_y; // Remember, this is reversed!
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        robot.fleft.setPower(-ly + lx + rx);
        robot.bleft.setPower(-ly - lx + rx);
        robot.fright.setPower(-ly - lx - rx);
        robot.bright.setPower(-ly + lx - rx);

        if (gamepad1.left_bumper && reverse) {
            reverse = false;
        } else if (gamepad1.right_bumper && !reverse) {
            reverse = true;
        }

        if (!reverse) {
            robot.fleft.setPower(-ly + lx + rx);
            robot.bleft.setPower(-ly - lx + rx);
            robot.fright.setPower(-ly - lx - rx);
            robot.bright.setPower(-ly + lx - rx);
        }
        if (reverse) {
            robot.fleft.setPower(ly - lx - rx);
            robot.bleft.setPower(ly + lx - rx);
            robot.fright.setPower(ly + lx + rx);
            robot.bright.setPower(ly - lx + rx);
        }

        if (-gamepad1.right_stick_y > 0) {
            robot.lift.setPower(-0.9);
        } else if (-gamepad1.right_stick_y < 0) {
            robot.lift.setPower(0.9);
        } else {
            robot.lift.setPower(0);
        }

        if (gamepad1.right_trigger > 0){
            robot.extendor.setPower(0.1);
        } else if (gamepad1.left_trigger > 0){
            robot.extendor.setPower(-0.1);
        } else {
            robot.extendor.setPower(0);
        }

        if (gamepad1.right_bumper){
            robot.claw.setPosition(0.7);
        } else if (gamepad1.left_bumper){
            robot.claw.setPosition(0.3);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
