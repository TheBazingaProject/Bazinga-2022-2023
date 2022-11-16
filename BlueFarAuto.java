package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "BlueAudienceCamMove", group = "Iterative Opmode")
//@Disabled
public class BlueFarAuto extends OpMode {

    // Declare OpMode members.
    Hardwaremap robot = new Hardwaremap();
    ElapsedTime runtime = new ElapsedTime();
    Movement movement = new Movement();


    String task = "Start";

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck"
    };
    public int BarcodePosition = 1;
    private WebcamName webcamName       = null;

    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AatjoPH/////AAABmUVCe9GkqkgonwUZ+sukJtU/jOe4gIlnzYqC/xWax9QPT/IvgEVwcy5TfmzM8mkSdpIUv+NSgcQlgFNtErVYWldcLjo8JzSGR7zLO7JH/cu1/OY/S4JzokvXRk4Kg4TvbsvuU7mskkmYqLDZ8F1TYgLbEDHqy57yCy/umU/CXAwS5OGioN6WC5P8x7btlNt8Vitmlcp1aQ1Ru8bdQo0742/0WBNp0Si3MEwJzRQUCFBOhNz+jzWoRxg6GKiQW5452gjMBl/guKVYtCuoZdiAkHjy8+/CzS3yF+9c6eLe8JAIgL0/Wq568IQO/Q4lS3K/k6GZP6jh/cjvZOlnWOscm9kZ2F4R5a1Q6/eoBvzWUk2l";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    boolean first = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {

        robot.init(hardwareMap);
        movement.init(robot);

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
////      parameters.loggingEnabled = false;
//
//        robot.imu.initialize(parameters);
//        while (!robot.imu.isGyroCalibrated()) {
//        }
//        telemetry.addData("Imu Calibration Status:", robot.imu.getCalibrationStatus());
//        telemetry.update();
//
//        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized :)");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

//        if (tfod != null) {
//            // getUpdatedRecognitions() will return null if no new information is available since
//            // the last time that call was made.
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                telemetry.addData("# Object Detected", updatedRecognitions.size());
//                if (updatedRecognitions.size() == 0) {
//                    BarcodePosition = 1;
//                    telemetry.addData("Barcode Position", BarcodePosition);
//                }
//                // step through the list of recognitions and display boundary info.
//                int i = 0;
//                boolean isDuckDetected = false;
//                for (Recognition recognition : updatedRecognitions) {
//                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                            recognition.getLeft(), recognition.getTop());
//                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                            recognition.getRight(), recognition.getBottom());
//                    i++;
//
//                    // check label to see if the camera sees a Duck
//                    if (recognition.getLabel().equals("Ball")) {
//                        isDuckDetected = true;
//                        telemetry.addData("Object Detected", "Capstone");
//                        if (recognition.getLeft() > 355 && recognition.getRight() > 400) {
//                            BarcodePosition = 3;
//                        } else if (recognition.getLeft() > 0 && recognition.getLeft() < 350 && recognition.getRight() > 180 && recognition.getRight() < 400) {
//                            BarcodePosition = 2;
//                        }
//                        telemetry.addData("Barcode Position", BarcodePosition);
//                    } else if (recognition.getLabel().equals("Duck")) {
//                        isDuckDetected = true;
//                        telemetry.addData("Object Detected", "Duck");
//                        if (recognition.getLeft() > 355 && recognition.getRight() > 400) {
//                            BarcodePosition = 3;
//                        } else if (recognition.getLeft() > 0 && recognition.getLeft() < 350 && recognition.getRight() > 180 && recognition.getRight() < 400) {
//                            BarcodePosition = 2;
//                        }
//                        telemetry.addData("Barcode Position", BarcodePosition);
//                    } else if (updatedRecognitions.size() == 0) {
//                        BarcodePosition = 1;
//                        telemetry.addData("Barcode Position", BarcodePosition);
//                    } else {
//                        isDuckDetected = false;
//                        telemetry.addData("Barcode Position", 1);
//                    }
//                }
//                telemetry.update();
//            }
//        }

        double cutOff = 0;
//        for (Recognition r : tfod.getRecognitions()) {
//            telemetry.addData("height", Math.abs(r.getBottom() - r.getTop()) + 5);
//        }
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void loop() {
        switch (task) {
            case "Start":
                runtime.reset();
                //scan signal using camera, save value as a variable so we can recall it later
                task = "drive forward";
                break;

            case "optional pause":
                //runtime.reset();
                //depending on our alliance's autonomous, pause a little so we dont run into them
                if (runtime.seconds() < 3) {
                    movement.drive(0,0);
                }
                task = "drive forward";
                break;

            case "drive forward":
                runtime.reset();
                //drive forward approximately 2 tiles to line up to middle goal
                task = "strafe to the right";
                break;

            case "strafe to the right":
                runtime.reset();
                //strafe approximatley one half of a tile to the right, facing the middle goal
                task = "deposit preload";
                break;

            case "deposit preload":
                runtime.reset();
                //raise the linear slide to the medium junction's height, 20 inches i think
                //move forward above junction
                // loosen the servo to drop it
                //return linear slide to neutral position
                //move back
                task = "strafe to the left";
                break;

            case "strafe to the left":
                runtime.reset();
                //strafe to the left around 3 tiles, aligned to the stack
                task = "move to stack";
                break;

            case "move to stack":
                runtime.reset();
                //move approximately half of a tile forward to reach stack
                task = "pick up cone";
                break;

            case "pick up cone":
                runtime.reset();
                //raise linear slide to top of 5 cone stack
                //move above stack
                //lower servo over cone
                //tighten servo
                //raise linear slide
                //move back
                task = "move back";
                break;

            case "move back":
                runtime.reset();
                //drive backwards around a tile and a half
                task = "drive up to junction";
                break;

            case "strafe up to junction":
                runtime.reset();
                //strafe to the right around 3 tiles to reach high junction
                task = "turn to high junction again";
                break;

            case "move to high junction":
                runtime.reset();
                //move around half a tile to reach high junction
                task = "deposit high junction";
                break;

            case "deposit high junction":
                runtime.reset();
                //raise up linear slide to high junction, around 30 inches
                //move above junction
                //loosen servo to drop cone
                //move back
                //return linear slide to neutral
                task = "move to park";
                break;

            case "move to park":
                runtime.reset();
                //rotate around 90 degrees
                //if sheldon is blue, move around 3 tiles forward
                //if sheldon is green, move around 2 tiles forward
                //if sheldon is red,  move around a tile forward
                task = "stop";
                break;

            case "stop":
                if (tfod != null) {
                    tfod.shutdown();
                }
                runtime.reset();
                stop();
                break;
        }

        telemetry.addData("task: ", task);
        telemetry.addData("sec:  ", runtime.seconds());
     //   telemetry.addData("angle", movement.getAngle());
        telemetry.addData("bleft stopped: ", robot.bleft.isBusy());
        telemetry.addData("bright stopped: ", robot.bright.isBusy());
        telemetry.addData("fleft stopped: ", robot.fleft.isBusy());
        telemetry.addData("fright stopped: ", robot.fright.isBusy());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//TODO        parameters.cameraName = Hardware.get(WebcamName.class, "Webcam1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     private void initTfod() {
     int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
     "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
     TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
     tfodParameters.minimumConfidence = 0.8;
     tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
     tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
     }
     */
}
