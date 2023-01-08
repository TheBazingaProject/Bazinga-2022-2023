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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@TeleOp(name="Robot: Teleop", group="Robot")

public class TeleopChosenOne extends OpMode{

    /* Declare OpMode members. */
    ElapsedTime runtime = new ElapsedTime();
    Hardwaremap robot = new Hardwaremap();
    Movement movement = new Movement();
    boolean reverse = false;
    boolean clawClosed = true;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     LIFT_WHEEL_DIAMETER_IN  = 2.0 ;
    static final double     LIFT_SPEED              = 1.0 ;
    static final double     LIFT_COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (LIFT_WHEEL_DIAMETER_IN * 3.1415);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Send telemetry message to signify robot waiting;
        robot.init(hardwareMap);
        runtime.reset();
        robot.claw.setPosition(0.6);
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
        double ly = gamepad1.left_stick_y; // Remember, this is reversed!
        double lx = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (gamepad1.left_bumper && reverse) {
            reverse = false;
        } else if (gamepad1.right_bumper && !reverse) {
            reverse = true;
        }

        if (!reverse) {
            robot.fleft.setPower(-ly - rx - lx);
            robot.bleft.setPower(-ly - rx + lx);
            robot.fright.setPower(-ly + rx + lx);
            robot.bright.setPower(-ly + rx - lx);
        }
        if (reverse) {
            robot.fleft.setPower(ly + rx + lx);
            robot.bleft.setPower(ly + rx - lx);
            robot.fright.setPower(ly - rx - lx);
            robot.bright.setPower(ly - rx + lx);
        }

        if (-gamepad2.right_stick_y > 0.8) {
            robot.liftL.setPower(0.85);
            robot.liftR.setPower(0.85);
        } else if (-gamepad2.right_stick_y < -0.8) {
            robot.liftL.setPower(-0.2);
            robot.liftR.setPower(-0.2);
        } else {
            robot.liftL.setPower(.2);
            robot.liftR.setPower(.2); //enough to fight gravity but not enough to cause it to accelerate
        }

        if (-gamepad2.left_stick_y > 0){
            robot.extendor.setPower(0.5);
        } else if (-gamepad2.left_stick_y < 0){
            robot.extendor.setPower(-0.5);
        } else {
            robot.extendor.setPower(0.06); //enough to keep it in place
        }

        if (robot.color2.blue() >= 20 || robot.color2.red() >= 20 && clawClosed){
            robot.claw.setPosition(0.2);
        }
//            if (gamepad2.left_bumper && clawClosed == true) {
//                robot.claw.setPosition(0.6);
//                clawClosed = false;
//            } else if (clawClosed == false) {
//                robot.claw.setPosition(0.2);
//                clawClosed = true;
//            }

        if (gamepad2.right_bumper){
            robot.claw.setPosition(0.2);
            clawClosed = false;
        } else if (gamepad2.left_bumper) {
            robot.claw.setPosition(0.6);
            clawClosed = true;
        }
//        } else if (robot.color2.blue() >= 20 || robot.color2.red() >= 20) { //autoclaw
//            robot.claw.setPosition(0.2);
//            movement.encoderMotor(robot.extendor, Movement.TURN_SPEED,15);
//       }
//
//        if (robot.claw.getPosition() >= .2) {
//            clawClosed = true;
//        }



        if (gamepad2.x){
            movement.encoderMotor(robot.liftL, movement.DRIVE_SPEED, 10);
            movement.encoderMotor(robot.liftR, movement.DRIVE_SPEED, 10);
            movement.encoderMotor(robot.extendor, movement.DRIVE_SPEED, 10);
        }   //makes extendor and lift instantly extend to max

        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Red", robot.color1.red() - 29);
        telemetry.addData("Green", robot.color1.green() - 52);
        telemetry.addData("Blue", robot.color1.blue() -42);
        telemetry.addData("Claw Vision: Blue", robot.color2.blue());
        telemetry.addData("Claw Vision: Red", robot.color2.red());
        telemetry.addData("ClawClosed", clawClosed);

        //telemetry.addData("claw distance", robot.distance.getDistance(DistanceUnit.INCH) + "inches");
        telemetry.addData("power", robot.claw.getConnectionInfo());
        telemetry.addData("Claw", robot.claw.getPosition());
        telemetry.addData("Velocity", robot.fleft.getVelocity());
        telemetry.update();

        //gamepad 1 right trigger continuoes servo intake

//        telemetry.addData("fleft", robot.fleft.getPower());
//        telemetry.addData("bleft", robot.bleft.getPower());
//        telemetry.addData("fright", robot.fright.getPower());
//        telemetry.addData("bright", robot.bright.getPower());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void extend(double speed, double extendInches) {
        int extendTarget;

        extendTarget = robot.extendor.getCurrentPosition() + (int)(extendInches * LIFT_COUNTS_PER_INCH);
        robot.extendor.setTargetPosition(extendTarget);

        robot.extendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extendor.setPower(Math.abs(speed));
    }
}

