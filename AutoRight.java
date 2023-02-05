package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Auto")
public class AutoRight extends LinearOpMode {
    Hardwaremap robot = new Hardwaremap();

    int zone = 2;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared Up
    static final double     LIFT_WHEEL_DIAMETER_IN  = 1.5 ;
    static final double     LIFT_COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (LIFT_WHEEL_DIAMETER_IN * 3.1415);

    public void senseColor(){
        if (robot.color1.blue() > robot.color1.red() && robot.color1.blue() > robot.color1.green()) {
            zone = 1;
        } else if (robot.color1.green() > robot.color1.blue() && robot.color1.green() > robot.color1.red()) {
            zone = 2;
        } else {
            zone = 3;
        }
        telemetry.addData("zone", zone);
        telemetry.update();
    }

//    public void closeClaw(){
//        robot.claw.setPosition(0.6);
//    }

    public void lifting(double speed) {
//        int liftTargetL;
//        int liftTargetR;
//
//        liftTargetL = robot.liftL.getCurrentPosition() + (int)(liftInches * LIFT_COUNTS_PER_INCH);
//        liftTargetR = robot.liftR.getCurrentPosition() + (int)(liftInches * LIFT_COUNTS_PER_INCH);
//        robot.liftL.setTargetPosition(liftTargetL);
//        robot.liftR.setTargetPosition(liftTargetR);
//
//        robot.liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftL.setPower(Math.abs(speed));
        robot.liftR.setPower(Math.abs(speed));
//
//        checkLift();
    }

    public void lift(double speed, double liftInches) {
        robot.liftL.setTargetPosition(robot.liftL.getCurrentPosition() + (int) (liftInches * LIFT_COUNTS_PER_INCH));
        robot.liftR.setTargetPosition(robot.liftR.getCurrentPosition() + (int) (liftInches * LIFT_COUNTS_PER_INCH));
        robot.liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftL.setPower(Math.abs(speed));
        robot.liftR.setPower(Math.abs(speed));
//        robot.liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.liftL.setPower(.025);
//        robot.liftR.setPower(.025);
//        robot.extendor.setPower(0.1);

    }

    public void liftTime(double speed){
        robot.liftR.setPower(Math.abs(speed));
        robot.liftL.setPower(Math.abs(speed));
    }

    public void liftOff(){
        robot.liftL.setPower(0.2);
        robot.liftR.setPower(0.2);
        robot.extendor.setPower(0.06);
    }


    public void extend(double speed, double extendInches) {
        int extendTarget;

        extendTarget = robot.extendor.getCurrentPosition() - (int)(extendInches * LIFT_COUNTS_PER_INCH);
        robot.extendor.setTargetPosition(extendTarget);

        robot.extendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extendor.setPower(Math.abs(speed));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);
        Pose2d startPose = new Pose2d(-34, -64.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence auto = drive.trajectorySequenceBuilder(startPose)
//                .forward(28)
//                .addTemporalMarker(() -> )
                .back(15,
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.waitSeconds(2)
                //.addTemporalMarker(() -> senseColor())
                .back(36)
                .forward(15)
                .lineToSplineHeading(new Pose2d(-37, -37, Math.toRadians(230)))

                .addTemporalMarker(() -> extend(0.5, 10))
                .waitSeconds(1.0)
                .back(9.25)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> robot.claw.setPosition(0.6))
                .waitSeconds(1)
                .addTemporalMarker(() -> robot.claw.setPosition(0.2))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extend(0.5, -9))
//                .waitSeconds(1)
//                //.splineToLinearHeading(new Pose2d(-32, -32), Math.toRadians(90))
//                .forward(7)
//                //.lineToLinearHeading(new Pose2d(-35, -36, Math.toRadians(315)))
//                .lineToLinearHeading(new Pose2d(-36, -5, Math.toRadians(180)))
//                // (-36, -13)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.claw.setPosition(0.6))
                .waitSeconds(10)
                .build();

        while(opModeInInit()){
            robot.claw.setPosition(0.2);
            robot.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        waitForStart();



        if (isStopRequested()) return;

        drive.followTrajectorySequence(auto);

        while(opModeIsActive()){
            telemetry.addData("targetL", robot.liftL.getTargetPosition());
            telemetry.addData("targetR", robot.liftR.getTargetPosition());
            telemetry.addData("currentL", robot.liftL.getCurrentPosition());
            telemetry.addData("currentR", robot.liftR.getCurrentPosition());
            telemetry.update();
        }
    }



}