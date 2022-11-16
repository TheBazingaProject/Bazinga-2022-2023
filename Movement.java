package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Movement {
    double globalAngle, power = .30, correction;
    Orientation lastAngles = new Orientation();
    double position = 0;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     MAX_REV                 = 300 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     LIFT_WHEEL_DIAMETER_IN  = 1.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     LIFT_COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (LIFT_WHEEL_DIAMETER_IN * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.8;
    static final double     LIFT_SPEED              = 0.8;

    Hardwaremap robot = new Hardwaremap();

    public void init(Hardwaremap mhwmap) {
        robot = mhwmap;
    }

//    public void lifting(double speed, double liftInches) {
//        int liftTarget;
//
//        liftTarget = robot.lift.getCurrentPosition() + (int)(liftInches * LIFT_COUNTS_PER_INCH);
//        robot.lift.setTargetPosition(liftTarget);
//
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(Math.abs(speed));
//    }

    public void encoderDrive(double speed, double rightInches, double leftInches) {
        int frightTarget;
        int fleftTarget;
        int brightTarget;
        int bleftTarget;

        // Determine new target position, and pass to motor controller
        fleftTarget = robot.fleft.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
        frightTarget = robot.fright.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
        bleftTarget = robot.bleft.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
        brightTarget = robot.bright.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);

        robot.fright.setTargetPosition(frightTarget);
        robot.fleft.setTargetPosition(fleftTarget);
        robot.bright.setTargetPosition(brightTarget);
        robot.bleft.setTargetPosition(bleftTarget);

        // Turn On RUN_TO_POSITION
        robot.fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        //runtime.reset();
        robot.fright.setPower(Math.abs(speed));
        robot.fleft.setPower(Math.abs(speed));
        robot.bright.setPower(Math.abs(speed));
        robot.bleft.setPower(Math.abs(speed));
    }

    // frightBleft is positive and fleftBright is negative to strafe left
    public void encoderStrafe(double speed,
                              double frightBleftInches, double fleftBrightInches) {
        int newFleftTarget;
        int newFrightTarget;
        int newBleftTarget;
        int newBrightTarget;

        // Determine new target position, and pass to motor controller
        newFleftTarget = robot.fleft.getCurrentPosition() + (int)(fleftBrightInches * COUNTS_PER_INCH);
        newFrightTarget = robot.fright.getCurrentPosition() + (int)(frightBleftInches * COUNTS_PER_INCH);
        newBleftTarget = robot.bleft.getCurrentPosition() + (int)(frightBleftInches * COUNTS_PER_INCH);
        newBrightTarget = robot.bright.getCurrentPosition() + (int)(fleftBrightInches * COUNTS_PER_INCH);

        robot.fright.setTargetPosition(newFrightTarget);
        robot.fleft.setTargetPosition(newFleftTarget);
        robot.bright.setTargetPosition(newBrightTarget);
        robot.bleft.setTargetPosition(newBleftTarget);

        // Turn On RUN_TO_POSITION
        robot.fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.fright.setPower(Math.abs(speed));
        robot.fleft.setPower(Math.abs(speed));
        robot.bright.setPower(Math.abs(speed));
        robot.bleft.setPower(Math.abs(speed));
    }
    //This is for diagonal strafing... before i knew it was built in
    public void encoderAngleStrafe(double speed,
                              double frightBleftInches, double fleftBrightInches, boolean right, boolean up) {
        int newFleftTarget;
        int newFrightTarget;
        int newBleftTarget;
        int newBrightTarget;

        // Determine new target position, and pass to motor controller
        newFleftTarget = robot.fleft.getCurrentPosition() + (int)(fleftBrightInches * COUNTS_PER_INCH);
        newFrightTarget = robot.fright.getCurrentPosition() + (int)(frightBleftInches * COUNTS_PER_INCH);
        newBleftTarget = robot.bleft.getCurrentPosition() + (int)(frightBleftInches * COUNTS_PER_INCH);
        newBrightTarget = robot.bright.getCurrentPosition() + (int)(fleftBrightInches * COUNTS_PER_INCH);

        robot.fright.setTargetPosition(newFrightTarget);
        robot.fleft.setTargetPosition(newFleftTarget);
        robot.bright.setTargetPosition(newBrightTarget);
        robot.bleft.setTargetPosition(newBleftTarget);

        // Turn On RUN_TO_POSITION
        robot.fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (up && right){
            robot.fright.setPower(0);
            robot.fleft.setPower(Math.abs(speed));
            robot.bright.setPower(Math.abs(speed));
            robot.bleft.setPower(0);
        }
        else if (!up && right){
            robot.fright.setPower(-(Math.abs(speed)));
            robot.fleft.setPower(0);
            robot.bright.setPower(0);
            robot.bleft.setPower(-(Math.abs(speed)));
        }
        else if (!up && !right){
            robot.fright.setPower(0);
            robot.fleft.setPower(-(Math.abs(speed)));
            robot.bright.setPower(-(Math.abs(speed)));
            robot.bleft.setPower(0);
        }
        else if (up && !right){
            robot.fright.setPower((Math.abs(speed)));
            robot.fleft.setPower(0);
            robot.bright.setPower(0);
            robot.bleft.setPower((Math.abs(speed)));
        }
        // reset the timeout time and start motion.

    }

    public boolean checkEncoderDone() {
        return !(robot.fright.isBusy() && robot.fleft.isBusy() && robot.bright.isBusy() && robot.bleft.isBusy());
    }

    public void encoderComplete(){
        robot.fright.setPower(0);
        robot.fleft.setPower(0);
        robot.bright.setPower(0);
        robot.bleft.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void resetEncoder() {
        position = robot.fleft.getCurrentPosition();
    }

    public double getPosition() {
        return robot.fleft.getCurrentPosition() - position;
    }

    public void drive(double leftPower, double rightPower) {
        robot.bleft.setPower(leftPower);
        robot.bright.setPower(rightPower);
        robot.fleft.setPower(leftPower);
        robot.fright.setPower(rightPower);
    }

//    public void resetAngle() {
//        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        globalAngle = 0;
//    }
//
//    /**
//     * Get current cumulative angle rotation from last reset.
//     *
//     * @return Angle in degrees. + = left, - = right.
//     */
//    public double getAngle() {
//        // We experimentally determined the Z axis is the axis we want to use for heading angle.
//        // We have to process the angle because the imu works in euler angles so the Z axis is
//        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
//    public double checkDirection() {
//        // The gain value determines how sensitive the correction is to direction changes.
//        // You will have to experiment with your robot to get small smooth direction changes
//        // to stay on a straight line.
//        double correction, angle, gain = .10;
//
//        angle = getAngle();
//
//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else
//            correction = -angle;        // reverse sign of angle for correction.
//
//        correction = correction * gain;
//
//        return correction;
//    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
//    public void rotate(int degrees, double power) {
//        double leftPower, rightPower;
//
//        // restart imu movement tracking.
//        // resetAngle();
//
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        // slow as we get closer
//        if (Math.abs(getAngle() - degrees) < 0.5) {
//            power = power * 0.0;
//        } else if (Math.abs(getAngle() - degrees) < 5) {
//            power = power * 0.25;
//        } else if (Math.abs(getAngle() - degrees) < 10) {
//            power = power * 0.5;
//        }
//
//        if (degrees < 0) {   // turn right.
//            leftPower = -power;
//            rightPower = power;
//        } else if (degrees > 0) {   // turn left.
//            leftPower = power;
//            rightPower = -power;
//        } else return;
//
//        // set power to rotate.
//        drive(leftPower, rightPower);
//
//        // turn the motors off.
//        //drive(0, 0);
//
//        // wait for rotation to stop.
//
//        // reset angle tracking on new heading.
//        // resetAngle();
//    }
}