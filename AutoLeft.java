package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Auto")
public class AutoLeft extends LinearOpMode {
    Hardwaremap robot = new Hardwaremap();
    int zone;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     COUNTS_PER_EXTEND_REV   = 1008   ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared Up
    static final double     LIFT_WHEEL_DIAMETER_IN  = 2.5 ;
    static final double     LIFT_COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (LIFT_WHEEL_DIAMETER_IN * 3.1415);
    static final double     EXTEND_COUNTS_PER_INCH    = (COUNTS_PER_EXTEND_REV * DRIVE_GEAR_REDUCTION) / (LIFT_WHEEL_DIAMETER_IN * 3.1415);

    public void senseColor(){
        if (robot.color1.blue() -42 > robot.color1.red() -20 && robot.color1.blue() -42 > robot.color1.green() -52) {
            zone = 1;
        } else if (robot.color1.red() - 20 > robot.color1.blue() -42 && robot.color1.red() -20 > robot.color1.green() -52) {
            zone = 3;
        } else  {
            zone = 2;
        }
    }

    public void lift(double speed, double liftInches) {
        int liftTarget;
        liftTarget = robot.liftL.getCurrentPosition() + (int) (liftInches * LIFT_COUNTS_PER_INCH);
        robot.liftL.setTargetPosition(liftTarget);
        robot.liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftL.setPower(Math.abs(speed));
        robot.liftR.setPower(.05);
    }

    public void resetEncoder() {
        robot.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void extend(double speed, double extendInches) {
        int extendTarget;

        extendTarget = robot.extendor.getCurrentPosition() - (int)(extendInches * EXTEND_COUNTS_PER_INCH);
        robot.extendor.setTargetPosition(extendTarget);

        robot.extendor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extendor.setPower(Math.abs(speed));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence auto = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.01)
                .addTemporalMarker(() -> {resetEncoder();})
                .lineToLinearHeading(new Pose2d(-35, -49, Math.toRadians(218)))
//                .back(10)
//                .turn(Math.toRadians(-49))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {extend(1, 9.05);}) // extends to low goal
                .waitSeconds(0.55)
                .addTemporalMarker(() -> robot.claw.setPosition(0.6)) //drops cone on low goal
                .waitSeconds(.3)
                .addTemporalMarker(() -> robot.claw.setPosition(0.2)) //closes claw
                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {extend(.5, -10);}) // retracts
                .turn(Math.toRadians(52))//turns straight
                .back(28)
                .addTemporalMarker(() -> {senseColor();}) // scans sheldon
                .lineToLinearHeading(new Pose2d(-36, -4, Math.toRadians(180)) )//turns toward conestack
//                .back(16)
//                .turn(Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> robot.claw.setPosition(0.6)) //opens claw
                .build();

        TrajectorySequence highGoal = drive.trajectorySequenceBuilder(auto.end())
                //.addTemporalMarker(() -> robot.claw.setPosition(0.6)) //open claw
                .lineToLinearHeading(new Pose2d(-62.7, -4.15, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))//to coneStack
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {lift(.9, 7);}) //lifts up to grab top cone
                .waitSeconds(.2)
                //.lineToConstantHeading(new Vector2d(-63, -4.2))// forward to cone stack
                .addTemporalMarker(() -> robot.claw.setPosition(0.2)) //closes claw
                .waitSeconds(.9)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {lift(1, 12);}) //lifts to max height
                //.lineToSplineHeading(new Pose2d(-38, -5, Math.toRadians(238)) )//gets out of the way and splines to high goal
                .lineToSplineHeading(new Pose2d(-34.8, -5 , Math.toRadians(233)))// to high goal
                .UNSTABLE_addTemporalMarkerOffset( -1, () -> {extend(.9, 13);}) //lifts to max height
                .waitSeconds(.65)
                .addTemporalMarker( () -> robot.claw.setPosition(0.6)) //drops cone on high goal
                .waitSeconds(.2)
                .addTemporalMarker(() -> robot.claw.setPosition(0.2)) //close claw
                //.turn(Math.toRadians(-62))
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {extend(1, -11.5);}) //retracts
                .UNSTABLE_addTemporalMarkerOffset(0.55,  () -> {lift(1, -20);}) //retracts
                .addTemporalMarker(() -> robot.claw.setPosition(0.6)) //opens claw
                .waitSeconds(.8)
                .build();

        TrajectorySequence greenPark = drive.trajectorySequenceBuilder(auto.end())
                .forward(2)
                .build();

        TrajectorySequence bluePark = drive.trajectorySequenceBuilder((auto.end()))
                .waitSeconds(.1)
                .lineToConstantHeading(new Vector2d(-60,-4))
                .build();

        TrajectorySequence redPark = drive.trajectorySequenceBuilder((auto.end()))
                .waitSeconds(.1)
                .lineToConstantHeading(new Vector2d(-12,-4))
                .build();

        while(opModeInInit()){
            robot.claw.setPosition(0.2);
        }

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(auto);
        telemetry.addData("zone", zone);
        telemetry.update();
        drive.followTrajectorySequence(highGoal);
        drive.followTrajectorySequence(highGoal);
        if (zone == 1) drive.followTrajectorySequence(bluePark);
        else if (zone == 2) drive.followTrajectorySequence(greenPark);
        else drive.followTrajectorySequence(redPark);
    }
}




