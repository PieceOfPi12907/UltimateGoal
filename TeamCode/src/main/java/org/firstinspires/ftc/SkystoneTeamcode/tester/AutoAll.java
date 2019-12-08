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
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Parking;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Repositioning;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDelivery;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "Autonomous All", group = "autonomous")

public class AutoAll extends LinearOpMode{

    Boolean isBlue = false;
    Boolean isOuter= false;
    Boolean isOneStone = false;
    Boolean isRepo = false;
    int delay = 0;


    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    BNO055IMU imu;
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
    Servo pivotGrabber;
    Servo blockClamper;
    Servo repositioningRight;
    Servo repositioningLeft;

    List<VuforiaTrackable> allTrackables;
    VuforiaLocalizer.Parameters parametersWebcam = null;
    final float mmPerInch = 25.4f;
    double y_value = 0;


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

    HashMap<String, Object> variableMap = new HashMap<String, Object>();


    public void initialize() {

        isBlue = false;
        isOuter= false;
        isOneStone = false;
        isRepo = false;
        delay = 0;


        telemetry.addLine("reached initialization");
        telemetry.update();

        //CHOOSE PROGRAM:
        while(true){
            //add delay parameter to gamepads!
            if(gamepad1.left_stick_button) {
                delay = delay + 5;
                telemetry.addLine("DELAY: " + delay + " seconds");
                telemetry.update();
            }

            if (gamepad1.right_stick_button) {
                delay = delay - 5;
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

            //bumper on gamepad 1 sets ONE STONE
            if(gamepad1.left_bumper){
                isOneStone = true;
                telemetry.addLine("ONE STONE");
                telemetry.update();
            }

            //bumper on gamepad 1 sets TWO STONES
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
            /*if(gamepad1.right_trigger > 0.1){
                isPark = true;
                telemetry.addLine("PARK ONLY");
                telemetry.update();
            }

            //left trigger for NO PARK
            if(gamepad1.left_trigger > 0.1){
                isPark = false;
                telemetry.addLine("NO PARK");
                telemetry.update();
            }*/

            //y on gamepad 1 CONFIRMS decisions
            if (gamepad1.y){
                telemetry.addData("DELAY: ", delay);
                telemetry.addData("COLOR: ", (isBlue == true)? "blue" : "red");
                telemetry.addData("ROUTE: ", (isOuter == true)? "outer" : "inner");
                telemetry.addData("STONES: ", (isOneStone == true)? "one stone" : "two stone");
                telemetry.addData("REPO: ", (isRepo == true)? "yes" : "no");
                telemetry.addLine("press 'a' to confirm!");
                telemetry.update();
            }

            if(gamepad1.a){
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

        //Setting the direction of the motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

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

        //frontColor=hardwareMap.get(ColorSensor.class,"frontColor");
        //backColor=hardwareMap.get(ColorSensor.class,"backColor");
        //quarryDistance=hardwareMap.get(DistanceSensor.class,"quarryDistance");

        //webcam = hardwareMap.get(WebcamName.class, "webcam");
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //parametersWebcam = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //webcamInitialization();

        //Initializing the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);
    }





    public Constants12907.SkystonePosition detectSkystoneWithWebcam(OpenGLMatrix lastLocation){//Telemetry pTelemetry, WebcamName pWebcam, VuforiaLocalizer.Parameters pParameters, ElapsedTime detectingTime, boolean blockFound, boolean targetVisible, OpenGLMatrix lastLocation, VectorF translation, float mmPerInch, double y_value, List<VuforiaTrackable> allTrackables, VuforiaTrackables targetsSkyStone ) {
        ElapsedTime detectingTime = new ElapsedTime();
        detectingTime.reset();
        boolean blockFound = false;
        boolean targetVisible = false;
        VectorF translation = null;
        while (detectingTime.seconds() <= 1 && !blockFound) {
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


    public OpenGLMatrix webcamInitialization (){
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parametersWebcam = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
        final boolean PHONE_IS_PORTRAIT = false;
        final String VUFORIA_KEY =
                "AU29nsP/////AAABmbmvuOwXSkEvjnmA/GE4vvtYcQ++rPSuF0c4fwBMDyTKVMiy0tgOzM+wgd2h5lhtywdhWMQRV+FPhX1SKEZ2LYwl1jKMN4JaKaWikc8DEfoXeFYf5cRAzlQa8CGQ2IFKgYm9Dq5tk8pdrD9WYqb4OFOUW6QkqhiOR1UCTQrAxgqCX0duHNRNK3ksVOyfDszUPL9r5nbIuaISyP5/iN7hWTbRk9damSem6xmKX4yex2YBroO0Ly7BX+JOiuu6x7c059WReN6DU1hrBDwUhIXxKjdV9OOTFL9uw1xedulivMI4G5LbjlQks09aSm/BbfUpCygx8oFo6NLikKP7V5RGUZBfOBwIP/cZDEb52gUZiBcp";
// Constant for Stone Target
        final float stoneZ = 2.00f * mmPerInch;
        OpenGLMatrix lastLocation = null;
        VuforiaLocalizer vuforia = null;
        float phoneXRotate = 0;
        float phoneYRotate = 0;
        float phoneZRotate = 0;
        boolean targetVisible = false;
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
        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
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

        VectorF translation = null;
        targetsSkyStone.activate();
        while (!opModeIsActive()) {
            targetsSkyStone.activate();
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
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
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                y_value = translation.get(1) / mmPerInch;

            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
        return lastLocation;
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
        variableMap.put(Constants12907.IMU, this.imu);

        variableMap.put(Constants12907.QUARRY_DISTANCE_SENSOR, this.quarryDistance);
        variableMap.put(Constants12907.FRONT_COLOR_SENSOR, this.frontColor);
        variableMap.put(Constants12907.BACK_COLOR_SENSOR, this.backColor);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Auto All entered RunOpMode");
        telemetry.update();

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

        //waitForStart();

        //if (opModeIsActive()) {

        createVariableMap();

        try {

            while(!isStopRequested() && !imu.isGyroCalibrated()){
                sleep(50);
                idle();
            }

            telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
            telemetry.update();

            OpenGLMatrix lastLocation = webcamInitialization();

            waitForStart();

            if (opModeIsActive()) {

                //delay set
                try {
                    Thread.sleep(delay);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if(!isRepo){
                    Constants12907.SkystonePosition skystonePosition = detectSkystoneWithWebcam(lastLocation);
                    if(skystonePosition.equals(Constants12907.SkystonePosition.LEFT)){
                        telemetry.addLine("LEFT");

                    }
                    if(skystonePosition.equals(Constants12907.SkystonePosition.RIGHT)){
                        telemetry.addLine("RIGHT");

                    }
                    if(skystonePosition.equals(Constants12907.SkystonePosition.CENTER)){
                        telemetry.addLine("CENTER");

                    }
                    telemetry.update();
                    variableMap.put(Constants12907.SKY_POSITION, skystonePosition);
                }

                if (isOneStone == false && isBlue == true && isOuter == false){
                    autoInnerTwoBlocksBlue.playProgram(variableMap);
                    telemetry.addLine("Program Playing: Blue Inner Two Block");
                }
            }

        }catch (Exception bad){
            telemetry.addData("EXCEPTION:", bad.toString());
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        //repositioning programs


        /*if (isRepo == true && isBlue == true && isOuter == true ){
            autoOuterRepoBlue.runOpMode();
            telemetry.addLine("Program Playing: Blue Outer Repo");
        }

        if (isRepo == true && isBlue == true && isOuter == false){
            autoInnerRepoBlue.runOpMode();
            telemetry.addLine("Program Playing: Blue Inner Repo");
        }

        if (isRepo == true && isBlue == false && isOuter == true){
            autoOuterRepoRed.runOpMode();
            telemetry.addLine("Program Playing: Red Outer Repo");
        }

        if (isRepo == true && isBlue == false && isOuter == false){
            autoInnerRepoRed.runOpMode();
            telemetry.addLine("Program Playing: Red Inner Repo");
        }

//one block programs

        if (isOneStone == true && isBlue == true && isOuter == true){
            autoOuterOneBlockRepoBlue.runOpMode();
            telemetry.addLine("Program Playing: Blue Outer One Block Repo");

        }

        if (isOneStone == true && isBlue == true && isOuter == false){
            autoInnerOneBlockRepoBlue.runOpMode();
            telemetry.addLine("Program Playing: Blue Inner One Block Repo");
        }

        if (isOneStone == true && isBlue == false && isOuter == true){
            autoOuterOneBlockRepoRed.runOpMode();
            telemetry.addLine("Program Playing: Red Outer One Block Repo");
        }

        if (isOneStone == true && isBlue == false && isOuter == false){
            autoInnerOneBlockRepoRed.runOpMode();
            telemetry.addLine("Program Playing: Red Inner One Block Repo");
        }

//two block programs

        if (isOneStone == false && isBlue == true && isOuter == false){
            autoInnerTwoBlocksBlue.runOpMode();
            telemetry.addLine("Program Playing: Blue Inner Two Block");
        }

        if (isOneStone == false && isBlue == false && isOuter == false){
            autoInnerTwoBlocksRed.runOpMode();
            telemetry.addLine("Program Playing: Red Inner Two Block");
        }

        //}*/
    }
}


