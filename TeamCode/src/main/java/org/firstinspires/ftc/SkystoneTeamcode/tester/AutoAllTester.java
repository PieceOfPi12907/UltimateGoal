package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoInnerOneBlockRepoBlue;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoInnerOneBlockRepoRed;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoInnerRepoBlue;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoInnerRepoRed;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoInnerTwoBlocksBlue;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoInnerTwoBlocksRed;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoOuterOneBlockRepoBlue;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoOuterOneBlockRepoRed;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoOuterRepoBlue;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoOuterRepoRed;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoParking;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.PIDController;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Parking;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Repositioning;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "Autonomous All TESTER", group = "autonomous")

public class AutoAllTester extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    ElapsedTime detection = new ElapsedTime();
    String position = "RIGHT";

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
            "AU29nsP/////AAABmbmvuOwXSkEvjnmA/GE4vvtYcQ++rPSuF0c4fwBMDyTKVMiy0tgOzM+wgd2h5lhtywdhWMQRV+FPhX1SKEZ2LYwl1jKMN4JaKaWikc8DEfoXeFYf5cRAzlQa8CGQ2IFKgYm9Dq5tk8pdrD9WYqb4OFOUW6QkqhiOR1UCTQrAxgqCX0duHNRNK3ksVOyfDszUPL9r5nbIuaISyP5/iN7hWTbRk9damSem6xmKX4yex2YBroO0Ly7BX+JOiuu6x7c059WReN6DU1hrBDwUhIXxKjdV9OOTFL9uw1xedulivMI4G5LbjlQks09aSm/BbfUpCygx8oFo6NLikKP7V5RGUZBfOBwIP/cZDEb52gUZiBcp";

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

    Boolean isPark = false;
    Boolean isBlue = false;
    Boolean isOuter= false;
    Boolean isOneStone = false;
    Boolean isRepo = true;
    long delay = 0;

    ElapsedTime runtime = new ElapsedTime();


    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    BNO055IMU imuBase;
    BNO055IMU imuSide;
    SkystoneDetection skystoneDetection;
    NavigationHelper navigationHelper;

    /*ColorSensor colorRight;
    DistanceSensor distanceRight;
    ColorSensor colorLeft;
    DistanceSensor distanceLeft;*/
    ColorSensor frontColor;
    ColorSensor backColor;

    DistanceSensor quarryDistance;
    WebcamName webcam;
    OpenGLMatrix lastLocation;
    WebcamThread webcamThread;

    Servo pivotGrabber;
    Servo blockClamper;
    Servo repositioningRight;
    Servo repositioningLeft;
    Servo slideServo;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    boolean targetVisible;
    VuforiaLocalizer.Parameters parametersWebcam = null;
    final float mmPerInch = 25.4f;
    double y_value = 0;

    TensorTesterClass tensor = new TensorTesterClass();

    AutoInnerOneBlockRepoBlue autoInnerOneBlockRepoBlue;
    AutoInnerOneBlockRepoRed autoInnerOneBlockRepoRed;
    AutoOuterOneBlockRepoBlue autoOuterOneBlockRepoBlue;
    AutoOuterOneBlockRepoRed autoOuterOneBlockRepoRed;

    AutoInnerTwoBlocksBlue autoInnerTwoBlocksBlue;
    AutoInnerTwoBlocksRed autoInnerTwoBlocksRed;

    AutoInnerRepoBlue autoInnerRepoBlue;
    AutoInnerRepoRed autoInnerRepoRed;
    AutoOuterRepoBlue autoOuterRepoBlue;
    AutoOuterRepoRed autoOuterRepoRed;

    AutoParking autoParking;

    Repositioning repositioning;



    HashMap<String, Object> variableMap = new HashMap<String, Object>();

    VuforiaTrackables targetsSkyStone;
    VectorF translation;


    boolean isDelayed = false;


    public void initialize() {

        isPark = false;
        isBlue = false;
        isOuter= false;
        isOneStone = false;
        isRepo = true;
        delay = 0;

        telemetry.addLine("reached initialization");
        telemetry.update();

        //CHOOSE PROGRAM:
        while(true){
            //add delay parameter to gamepads
            if((gamepad2.a) && (isDelayed == false)){
                isDelayed = true;
                delay += 5;
                /*if(delay > 10){
                    delay = 10;
                }*/
                telemetry.addLine("DELAY: " + delay + " seconds");
                telemetry.update();
            }

            if(gamepad2.y){
                isDelayed = false;
                telemetry.addLine("DELAY: " + delay + " seconds");
                telemetry.update();
            }

            if(gamepad2.x){
                delay = 0;
                telemetry.addLine("DELAY: " + delay + " seconds");
                telemetry.update();
            }

            //x on gamepad 1 sets BlUE
            if (gamepad1.x) {
                isBlue = true;
                telemetry.addLine("BLUE");
                telemetry.update();
            }

            //b on gamepad 1 sets RED
            if (gamepad1.b) {
                isBlue = false;
                telemetry.addLine("RED");
                telemetry.update();
            }

            //left dpad on gamepad 1 sets OUTER
            if(gamepad1.dpad_left){
                isOuter = true;
                telemetry.addLine("OUTER");
                telemetry.update();
            }

            //right dpad on gamepad 1 sets INNER
            if(gamepad1.dpad_right){
                isOuter = false;
                telemetry.addLine("INNER");
                telemetry.update();
            }

            //bumper left on gamepad 1 sets ONE STONE
            if(gamepad1.left_bumper){
                isOneStone = true;
                telemetry.addLine("ONE STONE");
                telemetry.update();
            }

            //bumper right on gamepad 1 sets TWO STONES
            if(gamepad1.right_bumper){
                isOneStone = false;
                telemetry.addLine("TWO STONE");
                telemetry.update();
            }
            //top dpad on gamepad 1 sets REPO
            if(gamepad1.dpad_up){
                isRepo = true;
                telemetry.addLine("REPO");
                telemetry.update();
            }

            //bottom dpad on gamepad 1 sets NO REPO!
            if(gamepad1.dpad_down){
                isRepo = false;
                telemetry.addLine("NO REPO");
                telemetry.update();
            }

            //right trigger for PARKING
            if(gamepad1.right_trigger > 0.1){
                isPark = true;
                telemetry.addLine("PARK ONLY");
                telemetry.update();
            }

            //left trigger for NO PARK
            if(gamepad1.left_trigger > 0.1){
                isPark = false;
                telemetry.addLine("NO PARK");
                telemetry.update();
            }

            //y on gamepad 1 CONFIRMS decisions
            if (gamepad1.a){
                telemetry.addData("DELAY: ", delay);
                telemetry.addData("COLOR: ", (isBlue == true)? "blue" : "red");
                telemetry.addData("ROUTE: ", (isOuter == true)? "outer" : "inner");
                telemetry.addData("STONES: ", (isOneStone == true)? "one stone" : "two stone");
                telemetry.addData("REPO: ", (isRepo == true)? "yes" : "no");
                telemetry.addData("ONLY PARK: ", (isPark == true)? "yes" : "no");
                telemetry.addLine("press 'y' to confirm!");
                telemetry.update();
            }

            if(gamepad1.y){
                telemetry.addLine("CONFIRMED");
                telemetry.update();
                break;
            }
        }

        //HARDWARE MAP:

        //Configuration of the Motors/Servos
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        pivotGrabber = hardwareMap.get(Servo.class, "pivotGrabber");
        blockClamper = hardwareMap.get(Servo.class, "blockClamper");
        repositioningRight = hardwareMap.get(Servo.class, "rightRepositioningServo");
        repositioningLeft = hardwareMap.get(Servo.class, "leftRepositioningServo");
        slideServo = hardwareMap.get(Servo.class,"slideServo");

        //Setting the direction of the motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //initialize more back so it's within the 18 by 18 limit
        pivotGrabber.setPosition(0.35);
        //blockClamper.setPosition(0.3);
        slideServo.setPosition(0.1);

        //braking
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initializing color and distance sensors
        /*colorRight=hardwareMap.get(ColorSensor.class,"sensor_color_distance_right");
        distanceRight=hardwareMap.get(DistanceSensor.class,"sensor_color_distance_right");
        colorLeft=hardwareMap.get(ColorSensor.class,"sensor_color_distance_left");
        distanceLeft=hardwareMap.get(DistanceSensor.class,"sensor_color_distance_left");*/

        frontColor=hardwareMap.get(ColorSensor.class,"frontColor");
        backColor=hardwareMap.get(ColorSensor.class,"backColor");
        quarryDistance=hardwareMap.get(DistanceSensor.class,"quarryDistance");

        webcam = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parametersWebcam = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //start up webcam
        //webcamInitialization();
        //Create thread
        //webcamThread = new WebcamThread();
        //Start thread
        //webcamThread.start();
        telemetry.addLine("webcam thread started");
        telemetry.update();

        //Initializing the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imuBase = hardwareMap.get(BNO055IMU.class, "imu");
        imuBase.initialize(parameters);
        imuSide = hardwareMap.get(BNO055IMU.class, "imu 1");
        imuSide.initialize(parameters);

    }

    public void webcamInitialization () {
        telemetry.addLine("running inside webcam thread");
        telemetry.update();
        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
        final boolean PHONE_IS_PORTRAIT = false;
        final String VUFORIA_KEY =
                "AU29nsP/////AAABmbmvuOwXSkEvjnmA/GE4vvtYcQ++rPSuF0c4fwBMDyTKVMiy0tgOzM+wgd2h5lhtywdhWMQRV+FPhX1SKEZ2LYwl1jKMN4JaKaWikc8DEfoXeFYf5cRAzlQa8CGQ2IFKgYm9Dq5tk8pdrD9WYqb4OFOUW6QkqhiOR1UCTQrAxgqCX0duHNRNK3ksVOyfDszUPL9r5nbIuaISyP5/iN7hWTbRk9damSem6xmKX4yex2YBroO0Ly7BX+JOiuu6x7c059WReN6DU1hrBDwUhIXxKjdV9OOTFL9uw1xedulivMI4G5LbjlQks09aSm/BbfUpCygx8oFo6NLikKP7V5RGUZBfOBwIP/cZDEb52gUZiBcp";
        // Constant for Stone Target
        final float stoneZ = 2.00f * mmPerInch;
        //OpenGLMatrix lastLocation = null;
        VuforiaLocalizer vuforia = null;
        float phoneXRotate = 0;
        float phoneYRotate = 0;
        float phoneZRotate = 0;
        targetVisible = false;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        parametersWebcam.vuforiaLicenseKey = VUFORIA_KEY;
        parametersWebcam.cameraName = webcam;

        vuforia = ClassFactory.getInstance().createVuforia(parametersWebcam);

// Load the data sets for the trackable objects. These particular data
// sets are stored in the 'assets' part of our application.
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");


        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");



        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.add(targetsSkyStone.get(0));


        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

// We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

// Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

// Next, translate the camera lens to where it is on the robot.
// In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parametersWebcam.cameraDirection);
        }

        translation = null;
        targetsSkyStone.activate();

    }


    public class WebcamThread extends Thread {
        volatile boolean stopThread = false;

        public void webcamThread() {
            this.setName("Webcam Thread");
        }

        public void requestStop(){
            stopThread = true;
            telemetry.addData("Stop Requested: ", stopThread);
            telemetry.update();

        }

        @Override
        public void run(){
            try {
                //while (!isInterrupted()) {
                while (!stopThread) {

                    targetsSkyStone.activate();
                    targetVisible = false;

                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            //telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;
                            targetsSkyStone.deactivate();

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }//for


                    // Provide feedback as to where the robot is located (if we know).
                    if (targetVisible) {
                        // express position (translation) of robot in inches.
                        translation = lastLocation.getTranslation();
                        //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        //translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);


                        y_value = translation.get(1) / mmPerInch;

                    } else {
                        //telemetry.addData("Visibsle Target", "none");
                    }
                    //telemetry.update();

                    //}
                }

                telemetry.addData("Stopped ", stopThread);
                telemetry.update();


            } catch (Exception E){

            }

        }
    }


    public Constants12907.SkystonePosition detectSkystoneWithWebcam(OpenGLMatrix lastLocation){//Telemetry pTelemetry, WebcamName pWebcam, VuforiaLocalizer.Parameters pParameters, ElapsedTime detectingTime, boolean blockFound, boolean targetVisible, OpenGLMatrix lastLocation, VectorF translation, float mmPerInch, double y_value, List<VuforiaTrackable> allTrackables, VuforiaTrackables targetsSkyStone ) {
        ElapsedTime detectingTime = new ElapsedTime();
        detectingTime.reset();
        boolean blockFound = false;
        boolean targetVisible = false;
        VectorF translation = null;
        while (detectingTime.seconds() <= 1.25 && !blockFound) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }//for

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                y_value = translation.get(1) / mmPerInch;
                blockFound = true;

            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();

        }
        Constants12907.SkystonePosition blockPosition = Constants12907.SkystonePosition.RIGHT;

        if (targetVisible == false) {
            blockPosition = Constants12907.SkystonePosition.LEFT;
            telemetry.addLine("********!!!!!!POS (LEFT)");
        } else if (y_value < 0) {
            blockPosition = Constants12907.SkystonePosition.CENTER;
            if (translation != null) {
                telemetry.addData("********!!!!!!POS (CENTER): ", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            }
        } else if (y_value > 0) {
            blockPosition = Constants12907.SkystonePosition.RIGHT;

            if (translation != null) {
                telemetry.addData("********!!!!!!POS (RIGHT): ", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            }
        }

        telemetry.update();

        return blockPosition;

    }


    private void createVariableMap(){
        variableMap.put(Constants12907.BLUE_FLAG, this.isBlue);
        variableMap.put(Constants12907.OUTER_FLAG, this.isOuter);

        variableMap.put(Constants12907.BACK_LEFT_MOTOR,this.backLeft);
        variableMap.put(Constants12907.FRONT_LEFT_MOTOR,this.frontLeft);
        variableMap.put(Constants12907.BACK_RIGHT_MOTOR,this.backRight);
        variableMap.put(Constants12907.FRONT_RIGHT_MOTOR,this.frontRight);

        variableMap.put(Constants12907.BLOCK_CLAMPER_SERVO, this.blockClamper);
        variableMap.put(Constants12907.PIVOT_GRABBER_SERVO, this.pivotGrabber);
        variableMap.put(Constants12907.RIGHT_REPOSITIONING_SERVO, this.repositioningRight);
        variableMap.put(Constants12907.LEFT_REPOSITIONING_SERVO, this.repositioningLeft);

        variableMap.put(Constants12907.TELEMETRY, this.telemetry);
        //variableMap.put(Constants12907.IMU, this.imu);
        variableMap.put(Constants12907.IMU, this.imuBase);

        variableMap.put(Constants12907.QUARRY_DISTANCE_SENSOR, this.quarryDistance);
        variableMap.put(Constants12907.FRONT_COLOR_SENSOR, this.frontColor);
        variableMap.put(Constants12907.BACK_COLOR_SENSOR, this.backColor);
        variableMap.put(Constants12907.WEBCAM, this.webcam);
        variableMap.put(Constants12907.PARAMETERS, this.parametersWebcam);

        variableMap.put(Constants12907.ELAPSEDTIME, this.runtime);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("RunOpmode Entered");
        telemetry.update();

        try {

            initialize();

            autoInnerOneBlockRepoBlue = new AutoInnerOneBlockRepoBlue();
            autoInnerOneBlockRepoBlue = new AutoInnerOneBlockRepoBlue();
            autoInnerOneBlockRepoRed = new AutoInnerOneBlockRepoRed();
            autoOuterOneBlockRepoBlue = new AutoOuterOneBlockRepoBlue();
            autoOuterOneBlockRepoRed = new AutoOuterOneBlockRepoRed();

            autoInnerTwoBlocksBlue = new AutoInnerTwoBlocksBlue();
            autoInnerTwoBlocksRed = new AutoInnerTwoBlocksRed();

            autoInnerRepoBlue = new AutoInnerRepoBlue();
            autoInnerRepoRed = new AutoInnerRepoRed();
            autoOuterRepoBlue = new AutoOuterRepoBlue();
            autoOuterRepoRed = new AutoOuterRepoRed();

            autoParking = new AutoParking();

            Repositioning repositioning = new Repositioning();
            Parking parkingClass = new Parking();
            SkystoneDetection skystoneDetection = new SkystoneDetection();

            createVariableMap();

            while(!isStopRequested() && !imuBase.isGyroCalibrated()){
                sleep(50);
                idle();
            }

            telemetry.addData("imu calib status: ", imuBase.getCalibrationStatus().toString());
            telemetry.update();


            waitForStart();

            if (opModeIsActive()) {
                runtime.reset();

                //delay set (if not set in init: delay is 0)
                sleep(delay * 1000);

                if (!isRepo) {

                    //calls right strafe method from this class (copied from navigationHelper) because with the thread running, we were unable to call the strafe right method from the class Navigation Helper
                        /*if(isBlue == true){
                        driveStraight(-4.5, -0.4, backLeft, backRight, frontRight, frontLeft, telemetry, imuBase);
                        }*/
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    //rightStrafeWithCorrection(9, 0.4, backLeft, backRight, frontRight, frontLeft, imuBase, telemetry);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    //webcamThread.requestStop();
                    //closes webcam
                    //webcamThread.interrupt();

                    //skystone position
                    Constants12907.SkystonePosition skystonePosition = detect();

                    rightStrafeWithCorrection(10, 0.4, backLeft, backRight, frontRight, frontLeft, imuBase, telemetry);

                    if(isBlue == true){
                        driveStraight(-4.5, -0.4, backLeft, backRight, frontRight, frontLeft, telemetry, imuBase);
                    }
                    if (skystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
                        telemetry.addLine("LEFT");

                    }
                    if (skystonePosition.equals(Constants12907.SkystonePosition.RIGHT)) {
                        telemetry.addLine("RIGHT");

                    }
                    if (skystonePosition.equals(Constants12907.SkystonePosition.CENTER)) {
                        telemetry.addLine("CENTER");

                    }
                    telemetry.update();


                    //inputs skystone position into the hashmap "variableMap"
                    variableMap.put(Constants12907.SKY_POSITION, skystonePosition);
                } else {
                    webcamThread.requestStop();
                    webcamThread.interrupt();

                }

                if (isPark == true) {
                    telemetry.addLine("Program Playing: Park");
                    telemetry.update();
                    parkingClass.moveToPark(frontLeft, frontRight, backLeft, backRight, navigationHelper, imuBase, telemetry, isBlue);
                    // autoParking.playProgram(variableMap);
                }
                if (isRepo == true && isBlue == true && isOuter == true && isPark == false) {
                    telemetry.addLine("Program Playing: Blue Outer Repo");
                    telemetry.update();
                    boolean isStoneRepo = false;
                    repositioning.doAngleRepositioning(frontLeft, frontRight, backLeft, backRight, navigationHelper, imuBase, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);
                    //autoOuterRepoBlue.playProgram(variableMap);
                } else if (isRepo == true && isBlue == true && isOuter == false && isPark == false) {
                    telemetry.addLine("Program Playing: Blue Inner Repo");
                    telemetry.update();
                    boolean isStoneRepo = false;
                    repositioning.doAngleRepositioning (frontLeft, frontRight, backLeft, backRight, navigationHelper, imuBase, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);
                    // autoInnerRepoBlue.playProgram(variableMap);
                } else if (isRepo == true && isBlue == false && isOuter == true && isPark == false) {
                    telemetry.addLine("Program Playing: Red Outer Repo");
                    telemetry.update();
                    boolean isStoneRepo = false;
                    repositioning.doAngleRepositioning(frontLeft, frontRight, backLeft, backRight, navigationHelper, imuBase, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);
                    // autoOuterRepoRed.playProgram(variableMap);
                } else if (isRepo == true && isBlue == false && isOuter == false && isPark == false) {
                    telemetry.addLine("Program Playing: Red Inner Repo");
                    telemetry.update();
                    boolean isStoneRepo = false;
                    repositioning.doAngleRepositioning(frontLeft, frontRight, backLeft, backRight, navigationHelper, imuBase, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);
                    //  autoInnerRepoRed.playProgram(variableMap);

                } else if (isRepo == false && isOneStone == true && isBlue == true && isOuter == true && isPark == false) {
                    telemetry.addLine("Program Playing: Blue Outer One Block Repo");
                    telemetry.update();
                    boolean isStoneRepo = true;
                    Constants12907.SkystonePosition position = (Constants12907.SkystonePosition) variableMap.get(Constants12907.SKY_POSITION);
                    skystoneDetection.moveToSkystoneOneWithREPO(backLeft, backRight, frontRight, frontLeft, navigationHelper, imuBase, telemetry, position, quarryDistance, pivotGrabber, blockClamper,  isBlue, repositioningRight,repositioningLeft);
                    repositioning.doAngleRepositioning(frontLeft, frontRight, backLeft, backRight, navigationHelper, imuBase, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);
                    //  autoOuterOneBlockRepoBlue.playProgram(variableMap);

                } else if (isRepo == false && isOneStone == true && isBlue == true && isOuter == false && isPark == false) {
                    telemetry.addLine("Program Playing: Blue Inner One Block Repo");
                    telemetry.update();
                    boolean isStoneRepo = true;
                    Constants12907.SkystonePosition position = (Constants12907.SkystonePosition) variableMap.get(Constants12907.SKY_POSITION);
                    skystoneDetection.moveToSkystoneOneWithREPO(backLeft, backRight, frontRight, frontLeft, navigationHelper, imuBase, telemetry, position, quarryDistance, pivotGrabber, blockClamper,  isBlue, repositioningRight, repositioningLeft);
                    repositioning.doAngleRepositioning(frontLeft, frontRight, backLeft, backRight, navigationHelper, imuBase, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);
                    //  autoInnerOneBlockRepoBlue.playProgram(variableMap);

                } else if (isRepo == false && isOneStone == true && isBlue == false && isOuter == true && isPark == false) {
                    telemetry.addLine("Program Playing: Red Outer One Block Repo");
                    telemetry.update();
                    boolean isStoneRepo = true;
                    Constants12907.SkystonePosition position = (Constants12907.SkystonePosition) variableMap.get(Constants12907.SKY_POSITION);
                    skystoneDetection.moveToSkystoneOneWithREPO(backLeft, backRight, frontRight, frontLeft, navigationHelper, imuBase, telemetry, position, quarryDistance, pivotGrabber, blockClamper,  isBlue,repositioningRight,repositioningLeft);
                    repositioning.doAngleRepositioning(frontLeft, frontRight, backLeft, backRight, navigationHelper, imuBase, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);
                    //autoOuterOneBlockRepoRed.playProgram(variableMap);

                } else if (isRepo == false && isOneStone == true && isBlue == false && isOuter == false && isPark == false) {
                    telemetry.addLine("Program Playing: Red Inner One Block Repo");
                    telemetry.update();
                    boolean isStoneRepo = true;
                    Constants12907.SkystonePosition position = (Constants12907.SkystonePosition) variableMap.get(Constants12907.SKY_POSITION);
                    skystoneDetection.moveToSkystoneOneWithREPO(backLeft, backRight, frontRight, frontLeft, navigationHelper, imuBase, telemetry, position, quarryDistance, pivotGrabber, blockClamper, isBlue,repositioningRight,repositioningLeft);
                    repositioning.doAngleRepositioning(frontLeft, frontRight, backLeft, backRight, navigationHelper, imuBase, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);
                    //autoInnerOneBlockRepoRed.playProgram(variableMap);

                } else if (isRepo == false && isOneStone == false && isBlue == true && isOuter == false && isPark == false) {
                    telemetry.addLine("Program Playing: Blue Inner Two Block");
                    telemetry.update();
                    Constants12907.SkystonePosition position = (Constants12907.SkystonePosition) variableMap.get(Constants12907.SKY_POSITION);
                    skystoneDetection.moveToSkystoneOne(backLeft, backRight, frontRight, frontLeft, navigationHelper, imuBase, telemetry, position, quarryDistance, pivotGrabber, blockClamper, isBlue.booleanValue());
                    skystoneDetection.moveToSkystoneTwo(backLeft, backRight, frontRight, frontLeft, navigationHelper, imuBase, telemetry, position, quarryDistance, pivotGrabber, blockClamper,  isBlue.booleanValue(), runtime);
                    //autoInnerTwoBlocksBlue.playProgram(variableMap);

                } else if (isRepo == false && isOneStone == false && isBlue == false && isOuter == false && isPark == false) {
                    telemetry.addLine("Program Playing: Red Inner Two Block");
                    telemetry.update();
                    Constants12907.SkystonePosition position = (Constants12907.SkystonePosition) variableMap.get(Constants12907.SKY_POSITION);
                    skystoneDetection.moveToSkystoneOne(backLeft, backRight, frontRight, frontLeft, navigationHelper, imuBase, telemetry, position, quarryDistance, pivotGrabber, blockClamper,  isBlue.booleanValue());
                    skystoneDetection.moveToSkystoneTwo(backLeft, backRight, frontRight, frontLeft, navigationHelper, imuBase, telemetry,position, quarryDistance, pivotGrabber, blockClamper, isBlue.booleanValue(), runtime);
                    //autoInnerTwoBlocksRed.playProgram(variableMap);
                }


            }
            webcam.close();

            //reset imu
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            imuBase.initialize(parameters);

            imuSide.initialize(parameters);

            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            // Disable Tracking when we are done;
            if(targetsSkyStone !=null){
                targetsSkyStone.deactivate();
            }

            telemetry.addLine("PROGRAM END");
            telemetry.update();


        } catch (Exception bad){
            telemetry.addData("EXCEPTION!!!:", bad.getMessage());
            bad.printStackTrace();
            telemetry.update();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        stop();

    } //RunOpmode


    //copied from naviagation helper for this class
    private void driveStraight (double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, Telemetry telemetry, BNO055IMU pImu) {
        ElapsedTime runtime = new ElapsedTime();

        PIDController pidDrive = new PIDController(.05, 0, 0);

        //Variables used for converting inches to Encoder dounts
        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        // newTargetPosition is the target position after it has been converted
        int newTargetPositionRight;
        int newTargetPositionLeft;
        // Sets all encoder values to 0 as we are moving robot with all 4 encoders
        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, converts it, and pass to motor controller
        newTargetPositionLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionLeft = pFrontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionRight = pFrontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(newTargetPositionLeft);
        pBackRight.setTargetPosition(newTargetPositionRight);
        pFrontLeft.setTargetPosition(newTargetPositionLeft);
        pFrontRight.setTargetPosition(newTargetPositionRight);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition());
        telemetry.addData("New Target Position","Left %7d : Right %7d", newTargetPositionLeft,newTargetPositionRight);
        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pFrontLeft.getCurrentPosition(), pFrontRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        pFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*pFrontRight.setPower((pSpeed));
        pBackRight.setPower((pSpeed));
        pFrontLeft.setPower((pSpeed));
        pBackLeft.setPower((pSpeed));*/

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, pSpeed);
        pidDrive.setInputRange(90, 90);
        pidDrive.enable();
        double correction;

        //This while loop will keep the motors running to the target position until one of the motors have reached the final encoder count
        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {
            correction = 0;
            pFrontRight.setPower(pSpeed+correction);
            pBackRight.setPower(pSpeed+correction);
            pFrontLeft.setPower(pSpeed-correction);
            pBackLeft.setPower(pSpeed-correction);
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d : %7d: %7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition(),
                pFrontLeft.getCurrentPosition(),
                pFrontRight.getCurrentPosition());
        telemetry.update();
    }

    private void rightStrafe(double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft,  BNO055IMU pImu, Telemetry telemetry) {
        ElapsedTime runtime = new ElapsedTime();

        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        int newTargetPositionFrontRight;
        int newTargetPositionFrontLeft;
        int newTargetPositionBackRight;
        int newTargetPositionBackLeft;

        double startingAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionBackRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontLeft = pFrontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontRight = pFrontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(-(newTargetPositionBackLeft));
        pBackRight.setTargetPosition(newTargetPositionBackRight);
        pFrontRight.setTargetPosition(-(newTargetPositionFrontRight));
        pFrontLeft.setTargetPosition(newTargetPositionBackLeft);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition(), pFrontLeft.getCurrentPosition(), pFrontRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pFrontRight.setPower(-(pSpeed));
        pBackRight.setPower((pSpeed));
        pFrontLeft.setPower((pSpeed));
        pBackLeft.setPower(-(pSpeed));

        telemetry.addData("Path1", "Target Position %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);
        telemetry.update();

        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {
            // Display it for the driver.
            /*telemetry.addData("Path1", "Running to %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    pBackLeft.getCurrentPosition(),
                    pBackRight.getCurrentPosition(),
                    pFrontLeft.getCurrentPosition(),
                    pFrontRight.getCurrentPosition());
            telemetry.update();*/
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition(),
                pFrontLeft.getCurrentPosition(),
                pFrontRight.getCurrentPosition());
        telemetry.update();
    }

    private void rightStrafeWithCorrection(double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft,  BNO055IMU pImu, Telemetry telemetry) {
        ElapsedTime runtime = new ElapsedTime();

        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        int newTargetPositionFrontRight;
        int newTargetPositionFrontLeft;
        int newTargetPositionBackRight;
        int newTargetPositionBackLeft;

        double startingAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionBackRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontLeft = pFrontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontRight = pFrontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(-(newTargetPositionBackLeft));
        pBackRight.setTargetPosition(newTargetPositionBackRight);
        pFrontRight.setTargetPosition(-(newTargetPositionFrontRight));
        pFrontLeft.setTargetPosition(newTargetPositionBackLeft);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition(), pFrontLeft.getCurrentPosition(), pFrontRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pFrontRight.setPower(-(pSpeed));
        pBackRight.setPower((pSpeed));
        pFrontLeft.setPower((pSpeed));
        pBackLeft.setPower(-(pSpeed));

        telemetry.addData("Path1", "Target Position %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);
        telemetry.update();

        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {

            // Display it for the driver.
            /*telemetry.addData("Path1", "Running to %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    pBackLeft.getCurrentPosition(),
                    pBackRight.getCurrentPosition(),
                    pFrontLeft.getCurrentPosition(),
                    pFrontRight.getCurrentPosition());
            telemetry.update();*/
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition(),
                pFrontLeft.getCurrentPosition(),
                pFrontRight.getCurrentPosition());
        telemetry.update();

        this.strafeCorrection(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, telemetry, startingAngle);


    }

    private void strafeCorrection (DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft,  BNO055IMU pImu, Telemetry pTelemetry, double pStartAngle) {

        double currentAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        double correction = pStartAngle-currentAngle;
        if(Math.abs(correction)>5){
            pTelemetry.addData("Correction value: ",correction);
            pTelemetry.update();
            turnWithEncoders(pFrontRight,pFrontLeft,pBackRight,pBackLeft, correction,0.15,pImu,pTelemetry);

        }

    }

    public void turnWithEncoders(DcMotor pFrontRight, DcMotor pFrontLeft, DcMotor pBackRight, DcMotor pBackLeft, double pRotation, double pSpeed, BNO055IMU pImu, Telemetry pTelemetry) {
        double computedAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        double currentAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        double previousAngle = 0.0;
        pTelemetry.addData("Initial Angle: ", computedAngle);
        pTelemetry.update();



        pFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (pRotation > 0) {
            while (computedAngle < pRotation) {
                //right
                previousAngle = computedAngle;
                pFrontRight.setPower(pSpeed);
                pBackRight.setPower(pSpeed);
                pFrontLeft.setPower(-pSpeed);
                pBackLeft.setPower(-pSpeed);
                computedAngle = ((pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES)).firstAngle) - currentAngle;
            }
        } else {
            while (computedAngle > pRotation) {
                //left
                previousAngle = computedAngle;
                pFrontRight.setPower(-pSpeed);
                pBackRight.setPower(-pSpeed);
                pFrontLeft.setPower(pSpeed);
                pBackLeft.setPower(pSpeed);
                computedAngle = ((pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES)).firstAngle) - currentAngle;
            }
        }
        pFrontRight.setPower(0);
        pBackRight.setPower(0);
        pFrontLeft.setPower(0);
        pBackLeft.setPower(0);
        currentAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        pTelemetry.addData("Previous Angle: ",previousAngle);
        pTelemetry.addData("Computed Angle: ",computedAngle);

        pTelemetry.addData("Current Angle: ",currentAngle);
        pTelemetry.update();
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public Constants12907.SkystonePosition detect(){
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        detection.reset();
        while (detection.seconds()<=1) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if(recognition.getLabel().equals("Skystone")) {
                              /*telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                              telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                      recognition.getLeft(), recognition.getTop());
                              telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                      recognition.getRight(), recognition.getBottom());

                               */
                            if(recognition.getLeft()<300){
                                telemetry.addData("Skystone position","LEFT");
                                position = "LEFT";
                            }
                            if(recognition.getRight()>500){
                                telemetry.addData("Skystone position","CENTER");
                                position = "CENTER";
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        if(position.equals("LEFT")){
            return Constants12907.SkystonePosition.LEFT;
        }
        else if(position.equals("CENTER")){
            return Constants12907.SkystonePosition.CENTER;
        }
        else{
            return Constants12907.SkystonePosition.RIGHT;
        }

    }

} //End of Class
