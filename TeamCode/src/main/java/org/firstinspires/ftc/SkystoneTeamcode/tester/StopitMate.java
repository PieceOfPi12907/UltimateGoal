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
import org.firstinspires.ftc.SkystoneTeamcode.opmode.OpenCvTester;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.PIDController;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Stop Tester Mate", group = "autonomous")

public class StopitMate extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    ElapsedTime detection = new ElapsedTime();


    double leftServoDown = 0.95;
    double rightServoDown = 0.01;

    double leftServoUp = 0.1;
    double rightServoUp = 0.85;
    SkystoneDetection skyStoneDetection = new SkystoneDetection();


    //String position = "RIGHT";

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

    Boolean isPark = true;
    Boolean isBlue = false;
    Boolean isOuter= false;
    Boolean isTwoStoneRepo = false;
    Boolean isRepo = false;
    long delay = 0;
    long parkDist = 0;

    ElapsedTime runtime = new ElapsedTime();

    DcMotor testMotor;
    BNO055IMU imu;
    Servo testServo;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    boolean targetVisible;
    VuforiaLocalizer.Parameters parametersWebcam = null;
    final float mmPerInch = 25.4f;
    double y_value = 0;

    VuforiaTrackables targetsSkyStone;
    VectorF translation;

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    boolean isDelayed = false;
    boolean isDistAdd = false;
    Constants12907.SkystonePosition skystonePosition;


    public void initialize() {

        isPark = false;
        isBlue = false;
        isOuter= false;
        isTwoStoneRepo = false;
        isRepo = true;
        delay = 0;
        parkDist = 0;

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

            if((gamepad2.dpad_up) && (isDistAdd == false)){
                isDistAdd = true;
                parkDist += 5;
                /*if(delay > 10){
                    delay = 10;
                }*/
                telemetry.addLine("Park Distance: " + parkDist + " inches");
                telemetry.update();
            }

            if(gamepad2.dpad_down){
                isDistAdd = false;
                telemetry.addLine("Park Distance: " + parkDist + " inches");
                telemetry.update();
            }

            if(gamepad2.dpad_left){
                parkDist = 0;
                telemetry.addLine("Park Distance: " + parkDist + " inches");
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

            //bumper right on gamepad 1 sets TWO STONE REPO
            if(gamepad1.right_bumper){
                isTwoStoneRepo = true;
                telemetry.addLine("TWO STONE REPO");
                telemetry.update();
            }

            //bumper left on gamepad 1 sets NO TWO STONE REPO
            if(gamepad1.left_bumper){
                isTwoStoneRepo = false;
                telemetry.addLine("NO TWO STONE REPO");
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
                telemetry.addData("Park Distance: ", parkDist);
                telemetry.addData("COLOR: ", (isBlue == true)? "blue" : "red");
                telemetry.addData("ROUTE: ", (isOuter == true)? "outer" : "inner");
                telemetry.addData("STONES: ", (isTwoStoneRepo == true)? "two stone + repo" : "no stones");
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
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        testServo = hardwareMap.get(Servo.class,"testServo");


        //Setting the direction of the motors
        testMotor.setDirection(DcMotor.Direction.REVERSE);

        //initialize stone grabbing arm more back so it's within the 18 by 18 limit

        testServo.setPosition(0.1);
        testServo.setPosition(0.85);
        if(isPark == true || isRepo == true){
            testServo.setPosition(0.8);
            testServo.setPosition(0.4);
        } else {
            testServo.setPosition(0.5);
            testServo.setPosition(0.5);
            testServo.setPosition(0.1);

        }

        //Initializing the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("RunOpmode Entered");
        telemetry.update();

        try {

            initialize();

            while(!isStopRequested() && !imu.isGyroCalibrated() && opModeIsActive()){
                sleep(50);
                idle();
            }

            telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
            telemetry.update();

            waitForStart();

            if (opModeIsActive()) {
                runtime.reset();

                //delay set (if not set in init: delay is 0)
                sleep(delay * 1000);

                if (!isRepo) {


                    //skystone position
                    skystonePosition = Constants12907.SkystonePosition.LEFT;

                    //inputs skystone position into the hashmap "variableMap"
                } else {

                }

                double direction;
                if(isBlue == true){
                    direction = 1;
                } else {
                    direction = -1;
                }

                if (isPark == true) {
                    telemetry.addLine("Program Playing: Park");
                    telemetry.update();
                    moveToPark();
                } else if (isRepo == true && isBlue == true && isOuter == true && isPark == false) {
                    telemetry.addLine("Program Playing: Blue Outer Repo");
                    telemetry.update();
                    doAngleRepositioning(direction,false);
                } else if (isRepo == true && isBlue == true && isOuter == false && isPark == false) {
                    telemetry.addLine("Program Playing: Blue Inner Repo");
                    telemetry.update();
                    doAngleRepositioning(direction,false);
                } else if (isRepo == true && isBlue == false && isOuter == true && isPark == false) {
                    telemetry.addLine("Program Playing: Red Outer Repo");
                    telemetry.update();
                    doAngleRepositioning(direction,false);
                } else if (isRepo == true && isBlue == false && isOuter == false && isPark == false) {
                    telemetry.addLine("Program Playing: Red Inner Repo");
                    telemetry.update();
                    doAngleRepositioning(direction,false);

                } else if(isTwoStoneRepo == true && isPark == false && isRepo == false){
                    //AutoTwoStoneRepo autoTwoStoneRepo = new AutoTwoStoneRepo();
                    telemetry.addLine("Program Playing: Two Stone Repo");
                    telemetry.update();
                    autoTwoStoneRepoPlayProgram();
                }
            }

            //try commenting out line 464 to 489

            //reset imu
            /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            imuBase.initialize(parameters);

            imuSide.initialize(parameters);*/

            testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            testMotor.setPower(0);


            // Disable Tracking when we are done;
            if(targetsSkyStone !=null){
                targetsSkyStone.deactivate();
            }

            telemetry.addLine("PROGRAM END");
            telemetry.update();

            idle();


        } catch (Exception bad){
            telemetry.addData("EXCEPTION!!!:", bad.getMessage());
            bad.printStackTrace();
            telemetry.update();
        }

        stop();

    } //RunOpmode



    public void moveToPark(){

        navigate(parkDist, Constants12907.Direction.STRAIGHT,0,0.5, true);

    }//moveToPark


    private void autoTwoStoneRepoPlayProgram(){


        Boolean isStoneRepo = true;

        try {


        /*    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/

            telemetry.addLine("********* PROGRAM RUNNING! ********** ");
            telemetry.update();

            double direction;

            if(isBlue == true){
                direction = 1;
            } else {
                direction = -1;
            }

            twoStonePlaceMethod(direction);
            doAngleRepositioning(direction,true);

        } catch (Exception bad){
            telemetry.addData("EXCEPTION:", bad.toString());
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }//autoTwoStoneRepoPlayProgram


    public void doAngleRepositioning(double direction, boolean isStoneRepo) {

        if(isStoneRepo != true){
            navigate(10*direction, Constants12907.Direction.STRAIGHT,0,0.5*direction,true);


            //rightStrafeWithoutCorrection(31, 0.75, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
            //rightStrafeWithoutCorrection(10, 0.75, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
            navigate(31, Constants12907.Direction.RIGHT,0,0.75,false);
            navigate(10, Constants12907.Direction.RIGHT,0,0.75,false);

            testServo.setPosition(leftServoDown);
            try {
                //500 --> 250
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            testServo.setPosition(rightServoDown);
            try {
                //750 --> 250
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if(isStoneRepo == true){
            //leftStrafeWithoutCorrection(20, 0.75, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            //navigate(20, Constants12907.Direction.LEFT,0,0.75,false);
            navigate(24, Constants12907.Direction.LEFT,0,0.9,false);
        } else {
            //leftStrafeWithoutCorrection(30, 0.75, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            navigate(30, Constants12907.Direction.LEFT,0,0.9,false);
        }



        testServo.setPosition(leftServoUp);
        try {
            //500 --> 250
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        testServo.setPosition(rightServoUp);
        try {
            //750 --> 250
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        navigate(10, Constants12907.Direction.LEFT,0,1.0,false);

    }//doAngleRepositioning


    private class SkystoneDropThread extends Thread {

        public void SkystoneDropThread() {
            this.setName("Attachments Thread");
        }

        @Override
        public void run() {

            try {
                skyStoneDetection.extakeSkystone(testServo, testServo);

            } catch (Exception e) {

            }
        }
    }


    private void twoStonePlaceMethod(double direction){

        //initializing variables:
        double firstStoneDistance = 0;
        double secondStoneDistance = 0;

        double leftDistance = -8;
        double centerDistance = -16;
        double rightDistance = -20;

        double imu_correct = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //ALIGN to skystone based on if it's in the left, center, or right position
        if(skystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            telemetry.addLine("position --> LEFT");

            if(isBlue){
                navigate(leftDistance, Constants12907.Direction.STRAIGHT, 0, -0.8,true);
                firstStoneDistance = 24;
                secondStoneDistance = 0;
            } else {
                navigate(rightDistance * direction, Constants12907.Direction.STRAIGHT, 0, -0.8 * direction,true);
                firstStoneDistance = 37;
                secondStoneDistance = 21;
            }
        }

        if(skystonePosition.equals(Constants12907.SkystonePosition.CENTER)) {
            telemetry.addLine("position --> CENTER");
            navigate(centerDistance*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction,true);

            firstStoneDistance = 32;
            secondStoneDistance = 8;
        }

        if(skystonePosition.equals(Constants12907.SkystonePosition.RIGHT)) {
            //pos from initial setup to reference point is 13
            telemetry.addLine("position --> RIGHT");
            //pNavigate.navigate(rightDistance*direction, Constants12907.Direction.STRAIGHT, 0, 00.8*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            if(isBlue){
                navigate((rightDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction, true);
                firstStoneDistance = 37;
                secondStoneDistance = 21;
            } else {
                navigate((leftDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction, true);
                firstStoneDistance = 24;
                secondStoneDistance = 0;
            }
        }


        //MOVE TO GRAB
            //instead of making a new method control using flag
        navigate(20,Constants12907.Direction.RIGHT,0,0.5,false);

        //get distance to move MORE towards quarry
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //based on the distance sensed, move a certain distance
        navigate(6,Constants12907.Direction.RIGHT,0,0.8,false);

        //angle correction

        //move MORE towards the wall if the skystone position is right on the blue side & left on the red side


        //GRAB skystone
        skyStoneDetection.intakeSkystone(testServo, testServo);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //strafe away from stones
        navigate(3.5,Constants12907.Direction.LEFT,0,0.5,false);

        //DELIVER skystone (to close side of foundation)
        //navigate((67 + firstStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction, true);
        navigate((65 + firstStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction, true);


        //angle correction

        //STRAFE to foundation with distance sensor
        navigate(15,Constants12907.Direction.RIGHT,0,0.25,false);

        //PLACE skystone
        skyStoneDetection.extakeSkystone(testServo, testServo);

        //STRAFE away from foundation
        navigate(15+0.6,Constants12907.Direction.LEFT,0,0.8,false);

        //angle correction

        //Navigate across field to get to second stone
        //navigate((-67 - secondStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, -0.6*direction, true);
        navigate((-65 - secondStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, -0.6*direction, true);

        //angle correction


        //MOVE towards second skystone in quarry using distance sensor


        //GRAB second skystone
        skyStoneDetection.intakeSkystone(testServo, testServo);

        //Strafe away from quarry
        navigate(3.25,Constants12907.Direction.LEFT,0,0.5,false);

        //angle correction


        //DELIVER second skystone (to close side of foundation)
        navigate((65 + secondStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction,true);

        //angle correction


        //set repo hooks to mid position (to move any stones on the foundation in the way)
        testServo.setPosition(0.45);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        testServo.setPosition(0.45);
        try {
            //500 --> 250
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //get distance to quarry

        //LATCH repositioning arms on to foundation
        testServo.setPosition(leftServoDown);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        testServo.setPosition(rightServoDown);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //PLACE second skystone
            //Trying out Thread instead of calling this directly
            //skyStoneDetection.extakeSkystone(blockClamper, pivotGrabber);
        Thread SkystoneDropThread = new SkystoneDropThread();
        SkystoneDropThread.start();

    } //twoStonePlaceMethod




//NAVIGATION HELPER &  OPEN CV METHODS BELOW

    private void navigate (double pTgtDistance, Constants12907.Direction pDirection, double pRotation, double pSpeed, boolean pCorrection){
        if(pDirection.equals(Constants12907.Direction.STRAIGHT)){
            forwardDrive(pTgtDistance, pSpeed);
        }

        else if(pDirection.equals(Constants12907.Direction.LEFT)){
            leftStrafe(pTgtDistance, pSpeed, pCorrection);

        }
        else if(pDirection.equals(Constants12907.Direction.RIGHT)){
            rightStrafe(pTgtDistance, pSpeed, pCorrection);
        }


    }//navigate


    Orientation lastAngles = new Orientation();
    double globalAngle;

    // This is the method that gets called if constant is STRAIGHT
    private void forwardDrive (double pTgtDistance, double pSpeed) {
        if(pSpeed<0){
            testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }else if(pSpeed>0){
            testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        ElapsedTime runtime = new ElapsedTime();

        PIDController pidDrive = new PIDController(.05, 0, 0);
        lastAngles = new Orientation();

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
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, converts it, and pass to motor controller
        newTargetPositionLeft = testMotor.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);

        testMotor.setTargetPosition(newTargetPositionLeft);

        runtime.reset();

        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, pSpeed);
        pidDrive.setInputRange(90, 90);
        pidDrive.enable();
        double correction;

        //This while loop will keep the motors running to the target position until one of the motors have reached the final encoder count
        while (testMotor.isBusy() && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            testMotor.setPower(pSpeed + correction);
        }

        //stop motors
        testMotor.setPower(0);

    }//forwardDrive


    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }//getAngle


    private void leftStrafe(double pTgtDistance, double pSpeed, boolean pCorrection) {
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

        double startingAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = testMotor.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);

        testMotor.setTargetPosition(newTargetPositionBackLeft);
        runtime.reset();

        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        testMotor.setPower((pSpeed));


        while (testMotor.isBusy() && opModeIsActive()) {
            // Display it for the driver.
        }

        //stop motors
        testMotor.setPower(0);


    }//leftStrafe

    private void rightStrafe(double pTgtDistance, double pSpeed, boolean pCorrection) {

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

        double startingAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = testMotor.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);

        testMotor.setTargetPosition(-(newTargetPositionBackLeft));

        runtime.reset();


        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        testMotor.setPower(-(pSpeed));

        while (opModeIsActive() && testMotor.isBusy() ) {

        }

        //stop motors
        testMotor.setPower(0);


    }//rightStrafe


} //End of Class
