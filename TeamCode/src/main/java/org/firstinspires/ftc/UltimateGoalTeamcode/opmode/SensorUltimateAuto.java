package org.firstinspires.ftc.UltimateGoalTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.Constants2020;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.DetectionHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.utilities.ShootingRings;
import org.firstinspires.ftc.UltimateGoalTeamcode.utilities.WobbleGoal;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;

@Autonomous(name = "ULTIMATE AUTO SENSOR", group = "autonomous")
public class SensorUltimateAuto extends LinearOpMode {

    Boolean isBlue = false;
    Boolean isWall = false;
    final double CLAMP_SERVO_IN = 0.2;
    final double HINGE_SERVO_UP = 0.1;
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    Servo wobbleHingeServo;
    Servo wobbleClampServo;
    OpenCvCamera webcam;
    ColorSensor frontColor;
    WobbleGoal wobbleGoal = new WobbleGoal();
    ShootingRings shootingRings = new ShootingRings();
    DetectionHelper pipeline;
    Constants2020.TargetZone box;
    HashMap<String, Object> variableMap = new HashMap<String, Object>();

    public void initialize() {

        isBlue = false;
        isWall = false;

        telemetry.addLine("reached initialization");
        telemetry.update();

        while(true) {
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

            //left dpad on gamepad 1 sets WALL
            if (gamepad1.dpad_left) {
                isWall = true;
                telemetry.addLine("WALL");
                telemetry.update();
            }

            //right dpad on gamepad 1 sets NOT WALL
            if (gamepad1.dpad_right) {
                isWall = false;
                telemetry.addLine("NOT WALL");
                telemetry.update();
            }

            //y on gamepad 1 CONFIRMS decisions
            if (gamepad1.a){
                telemetry.addData("COLOR: ", (isBlue == true)? "blue" : "red");
                telemetry.addData("ROUTE: ", (isWall == true)? "wall" : "not wall");
                telemetry.addLine("press 'y' to confirm!");
                telemetry.update();
            }

            if(gamepad1.y){
                telemetry.addLine("CONFIRMED");
                telemetry.update();
                break;
            }
        }

        //Initializing the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        telemetry.addLine("about to initialize webcam");
        telemetry.update();

        try{
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
            pipeline = new DetectionHelper();
            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                }
            });

        } catch (Exception bad){
            telemetry.addData("EXCEPTION (from try-catch):", bad.getMessage());
            bad.printStackTrace();
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        telemetry.addLine("FINISHED INITIALIZING WEBCAM");
        telemetry.update();
        try {
            Thread.sleep(2500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        frontColor = hardwareMap.get(ColorSensor.class, "frontColor");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        wobbleClampServo = hardwareMap.get(Servo.class, "clamp");
        wobbleHingeServo = hardwareMap.get(Servo.class, "hinge");

        //Setting the direction of the motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        createVariableMap();
    }

    private void createVariableMap(){
        variableMap.put(Constants2020.BLUE_FLAG, this.isBlue);
        variableMap.put(Constants2020.WALL_FLAG, this.isWall);

        variableMap.put(Constants2020.BACK_LEFT_MOTOR,this.backLeft);
        variableMap.put(Constants2020.FRONT_LEFT_MOTOR,this.frontLeft);
        variableMap.put(Constants2020.BACK_RIGHT_MOTOR,this.backRight);
        variableMap.put(Constants2020.FRONT_RIGHT_MOTOR,this.frontRight);

        variableMap.put(Constants2020.HINGE_SERVO,this.wobbleHingeServo);
        variableMap.put(Constants2020.CLAMP_SERVO,this.wobbleClampServo);

        variableMap.put(Constants2020.FRONT_COLOR, this.frontColor);
        //variableMap.put(Constants12907.IMU, this.imu);
        //MAKE SURE EDIT FROM 12907 class to 2020 class didn't change anything!!
        variableMap.put(Constants2020.IMU, this.imu);
        variableMap.put(Constants2020.WEBCAM, this.webcam);

        variableMap.put(Constants2020.ELAPSEDTIME, this.runtime);
        variableMap.put(Constants2020.TELEMETRY, this.telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("RunOpmode Entered");
        telemetry.update();

        try {

            initialize();

            while(!isStopRequested() && !imu.isGyroCalibrated()){
                sleep(50);
                idle();
            }

            telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
            telemetry.update();

            createVariableMap();

            DetectionHelper.RingPosition position = null;

            while(!opModeIsActive()) {

                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addData("Position", pipeline.getPosition());
                position = pipeline.getPosition();
                telemetry.update();
                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);
            }

            if (position.equals(DetectionHelper.RingPosition.NONE)){
                box = Constants2020.TargetZone.ALPHA;
            }
            if (position.equals(DetectionHelper.RingPosition.ONE)){
                box = Constants2020.TargetZone.BETA;
            }
            if (position.equals(DetectionHelper.RingPosition.FOUR)){
                box = Constants2020.TargetZone.CHARLIE;
            }

            variableMap.put(Constants2020.POSITION, this.box);
            waitForStart();

            if (opModeIsActive()) {
                runtime.reset();
                //int num = detectionLoop();
                telemetry.addData("Number of Rings", position);
                telemetry.update();
                sleep(500);

                wobbleGoal.moveToTgtZone(variableMap);
                //wobbleGoal.dropWobbleGoal(variableMap);
                shootingRings.moveToLaunchLine(variableMap);
            }

            //reset imu
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            imu.initialize(parameters);

            //initialize wobble arm
            wobbleClampServo.setPosition(CLAMP_SERVO_IN);
            wobbleHingeServo.setPosition(HINGE_SERVO_UP);

            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            telemetry.addLine("PROGRAM END");
            telemetry.update();


        } catch (Exception bad){
            telemetry.addData("EXCEPTION (from try-catch):", bad.getMessage());
            bad.printStackTrace();
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }


    }
    public int detectionLoop(){
        double begin = System.currentTimeMillis()/1000;
        double end = System.currentTimeMillis()/1000;
        DetectionHelper.RingPosition position = null;
        while(end-begin<1.5) {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.getPosition());
            position = pipeline.getPosition();
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            end = System.currentTimeMillis()/1000;
        }
        if(position.equals(DetectionHelper.RingPosition.FOUR)){
            return 4;
        }
        else if(position.equals(DetectionHelper.RingPosition.ONE)){
            return 1;
        }
        else if(position.equals(DetectionHelper.RingPosition.NONE)){
            return 0;
        }
        return 3;
    }
}
