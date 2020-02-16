package org.firstinspires.ftc.SkystoneTeamcode.opmode;

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
import org.firstinspires.ftc.SkystoneTeamcode.utillities.PIDController;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Parking;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Repositioning;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import java.util.HashMap;
import java.util.List;

@Autonomous(name = "non modular - AUTO ALL (opencv)", group = "autonomous")

public class AutoAllNonModularOpenCv extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    ElapsedTime detection = new ElapsedTime();


    double leftServoDown = 0.95;
    double rightServoDown = 0.01;

    double leftServoUp = 0.1;
    double rightServoUp = 0.85;


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


    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    BNO055IMU imuBase;
    BNO055IMU imuSide;


    ColorSensor frontColor;
    ColorSensor backColor;

    DistanceSensor quarryDistance;
    OpenCvCamera webcam;

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

        //initialize stone grabbing arm more back so it's within the 18 by 18 limit

        repositioningLeft.setPosition(0.1);
        repositioningRight.setPosition(0.85);
        if(isPark == true){
            blockClamper.setPosition(0.8);
            pivotGrabber.setPosition(0.4);
        } else {
            blockClamper.setPosition(0.5);
            pivotGrabber.setPosition(0.4);
            slideServo.setPosition(0.1);
        }


        frontColor=hardwareMap.get(ColorSensor.class,"frontColor");
        backColor=hardwareMap.get(ColorSensor.class,"backColor");
        quarryDistance=hardwareMap.get(DistanceSensor.class,"quarryDistance");



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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new OpenCvTester.StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);

    }



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("RunOpmode Entered");
        telemetry.update();

        try {

            initialize();

            while(!isStopRequested() && !imuBase.isGyroCalibrated() && opModeIsActive()){
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


                    //skystone position
                    skystonePosition = detectUsingOpenCV();

                    telemetry.addData("skystone position: ", skystonePosition);
                    telemetry.addData("val left = ", OpenCvTester.leftVal);
                    telemetry.addData("val mid = ",OpenCvTester.midVal);
                    telemetry.addData("val right = ", OpenCvTester.rightVal);
                    telemetry.update();

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
            webcam.closeCameraDevice();

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


            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

            repositioningLeft.setPosition(leftServoDown);
            try {
                //500 --> 250
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            repositioningRight.setPosition(rightServoDown);
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
            navigate(24, Constants12907.Direction.LEFT,0,0.75,false);
        } else {
            //leftStrafeWithoutCorrection(30, 0.75, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            navigate(30, Constants12907.Direction.LEFT,0,0.75,false);
        }

        turnWithEncoders(90*direction, 0.75);


        repositioningLeft.setPosition(leftServoUp);
        try {
            //500 --> 250
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningRight.setPosition(rightServoUp);
        try {
            //750 --> 250
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //rightStrafeWithoutCorrection(20,0.8,pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry );
        navigate(20, Constants12907.Direction.RIGHT,0,0.8,false);

        if (isOuter == true && runtime.seconds()<=28 && isStoneRepo == false) {

            //leftStrafeWithoutCorrection(5, 0.8, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            navigate(20, Constants12907.Direction.RIGHT,0,0.8,false);


            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            imuBase.initialize(parameters);

            //pNavigate.navigate(14 * direction, Constants12907.Direction.STRAIGHT, 0, 0.8 * direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            navigate(14 * direction, Constants12907.Direction.STRAIGHT, 0, 0.8 * direction, true);

            //leftStrafeWithoutCorrection(40, 0.99, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            navigate(40, Constants12907.Direction.LEFT,0,0.99,false);

        } else if ((isOuter == false || isStoneRepo == true) && runtime.seconds()<=28) {


            navigate(45, Constants12907.Direction.LEFT, 0, 0.99, true);

        }
        // }
    }//doAngleRepositioning



    private void twoStonePlaceMethod(double direction){
        double firstStoneDistance = 0;
        double secondStoneDistance = 0;
        double leftDistance = 12;
        //double centerDistance = 4;
        double centerDistance = 5;
        //double rightDistance = -1;
        double rightDistance = -4;
        double armOffset = 6;


        double imu_correct = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //ALIGN to skystone based on if it's in the left, center, or right position
        if(skystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            telemetry.addLine("position --> LEFT");
            if(isBlue){
                navigate(leftDistance, Constants12907.Direction.STRAIGHT, 0, 0.8,true);
            }else if (!isBlue){
                navigate((leftDistance-armOffset)-2, Constants12907.Direction.STRAIGHT, 0, 0.8,true);
            }


            if(isBlue){
                secondStoneDistance = 0;
                firstStoneDistance = 0;
            } else{
                secondStoneDistance = 20;
                firstStoneDistance = 17;
            }
        }
        if(skystonePosition.equals(Constants12907.SkystonePosition.CENTER)) {
            telemetry.addLine("position --> CENTER");
            navigate(centerDistance*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction,true);

            secondStoneDistance = 12;
            firstStoneDistance = 11;
        }
        if(skystonePosition.equals(Constants12907.SkystonePosition.RIGHT)) {
            telemetry.addLine("position --> RIGHT");
            //pNavigate.navigate(rightDistance*direction, Constants12907.Direction.STRAIGHT, 0, 00.8*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            if(isBlue){
                navigate(rightDistance, Constants12907.Direction.STRAIGHT, 0, 0.8, true);
            }
            else if(!isBlue){
                navigate(rightDistance-armOffset, Constants12907.Direction.STRAIGHT, 0, 0.8, true);
            }

            if(isBlue){
                secondStoneDistance = 20;
                firstStoneDistance = 17;
            } else {
                secondStoneDistance = 2;
                firstStoneDistance = 0;
            }

        }


        //MOVE TO GRAB using fixed distance for now to get to quarry
        //(47" (wall to quarry) - 18" (robot) = 29" (move) - 3" (gap btwn quarry & robot) = 26" (final))
        //pNavigate.navigate(26, Constants12907.Direction.RIGHT, 0, 00.8, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        //instead of making a new method control using flag
        //rightStrafeWithoutCorrection(20, 0.8,pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        navigate(20,Constants12907.Direction.RIGHT,0,0.8,false);
        double currentDistance1 = quarryDistance.getDistance(DistanceUnit.INCH);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(currentDistance1 > 7){
            double targetDistance = 6.5;
            double toGoDistance = currentDistance1 - targetDistance;
            //rightStrafeWithoutCorrection(toGoDistance, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            navigate(toGoDistance,Constants12907.Direction.RIGHT,0,0.25,false);
        } else if (currentDistance1 < 6){
            double targetDistance = 6.5;
            double toGoDistance = targetDistance - currentDistance1;
            //leftStrafeWithoutCorrection(toGoDistance, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            navigate(toGoDistance,Constants12907.Direction.LEFT,0,0.25,false);
        }

        double imuCorrection = (imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
        telemetry.addData("IMU CORRECTION: ", imuCorrection);
        telemetry.update();
        turnWithEncoders(-imuCorrection, 0.25);

        SkystoneDetection skyStoneDetection = new SkystoneDetection();
        //GRAB skystone
        skyStoneDetection.intakeSkystone(blockClamper, pivotGrabber);

        //leftStrafeWithoutCorrection(2, 0.5, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        navigate(2,Constants12907.Direction.LEFT,0,0.5,false);

        //DELIVER skystone (to far side of foundation)
        //pNavigate.navigate((78 + firstStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        navigate((67 + firstStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction, true);
        imuCorrection = (imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
        telemetry.addData("IMU CORRECTION: ", imuCorrection);
        telemetry.update();
        turnWithEncoders(-imuCorrection, 0.25);

        //STRAFE to foundation
        //rightStrafeWithoutCorrection(0, 0.8, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        double currentFoundationDistance1 = quarryDistance.getDistance(DistanceUnit.INCH);
        //rightStrafeWithoutCorrection(currentFoundationDistance1, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        navigate(currentFoundationDistance1,Constants12907.Direction.RIGHT,0,0.25,false);

        //PLACE skystone (on foundation - so it may require a strafe right before dropping, then a strafe left to come back to the same spot)
        skyStoneDetection.extakeSkystone(blockClamper, pivotGrabber);

        //STRAFE away from foundation
        //leftStrafeWithoutCorrection(currentFoundationDistance1+0.5, 0.8, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        navigate(currentFoundationDistance1+0.5,Constants12907.Direction.LEFT,0,0.8,false);

        imuCorrection = (imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
        telemetry.addData("IMU CORRECTION: ", imuCorrection);
        telemetry.update();
        turnWithEncoders(-imuCorrection, 0.25);


        //ALIGN back to second set of stones of quarry
        //pNavigate.navigate((-102 - secondStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, -0.8*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        navigate((-91 - secondStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, -0.8*direction, true);
        imuCorrection = (imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
        telemetry.addData("IMU CORRECTION: ", imuCorrection);
        telemetry.update();
        turnWithEncoders(-imuCorrection, 0.25);

        //MOVE to second skystone
        double currentDistance = quarryDistance.getDistance(DistanceUnit.INCH);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(currentDistance > 6.3){
            double targetDistance = 6;
            double toGoDistance = currentDistance - targetDistance;
            //rightStrafeWithoutCorrection(toGoDistance, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            navigate(toGoDistance,Constants12907.Direction.RIGHT,0,0.25,false);
        } else if (currentDistance < 6){
            double targetDistance = 6;
            double toGoDistance = targetDistance - currentDistance;
            //leftStrafeWithoutCorrection(toGoDistance, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            navigate(toGoDistance,Constants12907.Direction.LEFT,0,0.25,false);
        }


        imuCorrection = (imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
        telemetry.addData("IMU CORRECTION: ", imuCorrection);
        telemetry.update();
        turnWithEncoders(-imuCorrection, 0.25);

        //GRAB second skystone
        skyStoneDetection.intakeSkystone(blockClamper, pivotGrabber);

        //leftStrafeWithoutCorrection(2, 0.5, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        navigate(2,Constants12907.Direction.LEFT,0,0.5,false);

        //DELIVER second skystone (to close side of foundation)
        navigate((93 + secondStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.8*direction,true);
        imuCorrection = (imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
        telemetry.addData("IMU CORRECTION: ", imuCorrection);
        telemetry.update();
        turnWithEncoders(-imuCorrection, 0.25 );


        repositioningLeft.setPosition(0.45);
        try {
            //500 --> 2500
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        repositioningRight.setPosition(0.45);
        try {
            //500 --> 250
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        double currentFoundationDistance = quarryDistance.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance to Foundation",currentFoundationDistance);
        telemetry.update();


        //if(currentFoundationDistance > 2){
        //rightStrafeWithoutCorrection(currentFoundationDistance+1, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        navigate(currentFoundationDistance+1,Constants12907.Direction.RIGHT,0,0.25,false);
        telemetry.addData("Distance to Foundation After",quarryDistance.getDistance(DistanceUnit.INCH));
        //}


        //LATCH repositioning arms on to foundation
        repositioningLeft.setPosition(leftServoDown);
        try {
            //500 --> 2500
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        repositioningRight.setPosition(rightServoDown);
        try {
            //500 --> 250
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //PLACE second skystone
        skyStoneDetection.extakeSkystone(blockClamper, pivotGrabber);

    } //twoStonePlaceMethod





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

        else if(pDirection.equals(Constants12907.Direction.TURN)){
            turnWithEncodersWithCorrection(pRotation, pSpeed);

        }

    }//navigate

    Orientation lastAngles = new Orientation();
    double globalAngle;

    // This is the method that gets called if constant is STRAIGHT
    private void forwardDrive (double pTgtDistance, double pSpeed) {
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
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, converts it, and pass to motor controller
        newTargetPositionLeft = backLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionRight = backRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionLeft = frontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionRight = frontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        backLeft.setTargetPosition(newTargetPositionLeft);
        backRight.setTargetPosition(newTargetPositionRight);
        frontLeft.setTargetPosition(newTargetPositionLeft);
        frontRight.setTargetPosition(newTargetPositionRight);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        telemetry.addData("New Target Position","Left %7d : Right %7d", newTargetPositionLeft,newTargetPositionRight);
        telemetry.addData("Initial Value", "Running at %7d :%7d",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.update();

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, pSpeed);
        pidDrive.setInputRange(90, 90);
        pidDrive.enable();
        double correction;

        //This while loop will keep the motors running to the target position until one of the motors have reached the final encoder count
        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy() && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            frontRight.setPower(pSpeed + correction);
            backRight.setPower(pSpeed + correction);
            frontLeft.setPower(pSpeed - correction);
            backLeft.setPower(pSpeed - correction);
        }

        //stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d : %7d: %7d",
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition(),
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition());
        telemetry.update();

    }//forwardDrive


    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

        double startingAngle = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = backLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionBackRight = backRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontLeft = frontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontRight = frontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        backLeft.setTargetPosition(newTargetPositionBackLeft);
        backRight.setTargetPosition(-(newTargetPositionBackRight));
        frontRight.setTargetPosition(newTargetPositionFrontRight);
        frontLeft.setTargetPosition(-(newTargetPositionBackLeft));
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.update();

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower((pSpeed));
        backRight.setPower(-(pSpeed));
        frontLeft.setPower(-(pSpeed));
        backLeft.setPower((pSpeed));

        telemetry.addData("Path1", "Target Position %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);

        telemetry.update();

        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy() && opModeIsActive()) {
            // Display it for the driver.
        }

        //stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition(),
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition());
        telemetry.update();


        if(pCorrection) {
            this.strafeCorrection(startingAngle);
        }
    }//leftStrafe

    private void strafeCorrection (double pStartAngle) {

        double currentAngle = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        double correction = pStartAngle-currentAngle;
        if(Math.abs(correction)>5){
            telemetry.addData("Correction value: ",correction);
            telemetry.update();
            turnWithEncoders(correction,0.15);

        }

    }//strafeCorrection




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

        double startingAngle = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = backLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionBackRight = backRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontLeft = frontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontRight = frontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        backLeft.setTargetPosition(-(newTargetPositionBackLeft));
        backRight.setTargetPosition(newTargetPositionBackRight);
        frontRight.setTargetPosition(-(newTargetPositionFrontRight));
        frontLeft.setTargetPosition(newTargetPositionBackLeft);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.update();

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(-(pSpeed));
        backRight.setPower((pSpeed));
        frontLeft.setPower((pSpeed));
        backLeft.setPower(-(pSpeed));

        telemetry.addData("Path1", "Target Position %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);
        telemetry.update();

        while (opModeIsActive() && backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {

        }

        //stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition(),
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition());
        telemetry.update();

        if(pCorrection) {
            this.strafeCorrection(startingAngle);
        }

    }//rightStrafe


    public void turnWithEncoders(double pRotation, double pSpeed) {
        double computedAngle = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        double currentAngle = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        double previousAngle = 0.0;
        telemetry.addData("Initial Angle: ", computedAngle);
        telemetry.update();



        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (pRotation > 0) {
            while (computedAngle < pRotation && opModeIsActive()) {
                //right
                previousAngle = computedAngle;
                frontRight.setPower(pSpeed);
                backRight.setPower(pSpeed);
                frontLeft.setPower(-pSpeed);
                backLeft.setPower(-pSpeed);
                computedAngle = ((imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES)).firstAngle) - currentAngle;
            }
        } else {
            while (computedAngle > pRotation && opModeIsActive()) {
                //left
                previousAngle = computedAngle;
                frontRight.setPower(-pSpeed);
                backRight.setPower(-pSpeed);
                frontLeft.setPower(pSpeed);
                backLeft.setPower(pSpeed);
                computedAngle = ((imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES)).firstAngle) - currentAngle;
            }

        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        currentAngle = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        telemetry.addData("Previous Angle: ",previousAngle);
        telemetry.addData("Computed Angle: ",computedAngle);

        telemetry.addData("Current Angle: ",currentAngle);
        telemetry.update();
    }//turnWithEncoders

    private void turnWithEncodersWithCorrection(double pRotation, double pSpeed) {
        turnWithEncoders(pRotation,pSpeed);
        double currentAngle = imuBase.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        double correction = pRotation-currentAngle;
        if(Math.abs(correction)>=5){
            telemetry.addData("Correction value: ",correction);
            telemetry.update();

            turnWithEncoders(correction,0.15);

        }
    }//turnWithEncodersWithCorrection

























    public Constants12907.SkystonePosition detectUsingOpenCV(){

        int index = returnSkystoneIndex();
        telemetry.addData("Index: ",index);
        telemetry.update();

        if (index == 0){
            return Constants12907.SkystonePosition.LEFT;

        }
        else if (index == 1){
            return Constants12907.SkystonePosition.CENTER;

        }
        else {
            return Constants12907.SkystonePosition.RIGHT;

        }


    }

    public int returnSkystoneIndex() {

        if( OpenCvTester.leftVal== 0) {
            return 0;
        } else if(OpenCvTester.midVal == 0) {
            return 1;
        } else {
            return 2;
        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private OpenCvTester.StageSwitchingPipeline.Stage stageToRenderToViewport = OpenCvTester.StageSwitchingPipeline.Stage.detection;
        private OpenCvTester.StageSwitchingPipeline.Stage[] stages = OpenCvTester.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];


            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];


            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            /*telemetry.addData("valLeft,valMid,valRight ",valLeft+" "+valMid+" "+valRight);
            telemetry.update();
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }  */

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }


} //End of Class
