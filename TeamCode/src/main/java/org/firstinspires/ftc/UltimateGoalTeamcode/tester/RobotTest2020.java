package org.firstinspires.ftc.UltimateGoalTeamcode.tester;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;

import org.firstinspires.ftc.SkystoneTeamcode.tester.WebcamTester;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Control Hub Servo Test 1", group = "teleop")
public class RobotTest2020 extends LinearOpMode {

    //sensors
    ColorSensor bottomColor;
    DistanceSensor frontDistance;
    WebcamName webcam;
    WebcamTester webcamTester;
    BNO055IMU imu;

    //servos
    Servo wobbleServo;

    //motors
    DcMotor shootingMotor;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;

    private void initialize(){
        //sensors
        bottomColor = hardwareMap.get(ColorSensor.class,"bottomcolor");
        frontDistance = hardwareMap.get(DistanceSensor.class,"frontdistance");
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //servos
        wobbleServo = hardwareMap.get(Servo.class,"wobbleservo");

        //motors
        shootingMotor =hardwareMap.get(DcMotor.class,"shootingmotor");
        frontRightMotor =hardwareMap.get(DcMotor.class,"frontright");
        frontLeftMotor =hardwareMap.get(DcMotor.class, "frontleft");
        backRightMotor =hardwareMap.get(DcMotor.class,"backright");
        backLeftMotor =hardwareMap.get(DcMotor.class,"backleft");

        //setting motor initializations
        shootingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        webcamTester = new WebcamTester();

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }
        telemetry.addData("imu calibration status: ", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //color sensor
            if(gamepad1.a){
                float hsvValues[] = {0F, 0F, 0F};
                final double SCALE_FACTOR = 1;
                Color.RGBToHSV((int) (bottomColor.red() * SCALE_FACTOR),
                        (int) (bottomColor.green() * SCALE_FACTOR),
                        (int) (bottomColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("HUE: ", hsvValues[0]);
                telemetry.addData("RED: ", bottomColor.red());
                telemetry.addData("GREEN: ", bottomColor.green());
                telemetry.addData("BLUE:  ", bottomColor.blue());
                telemetry.update();
                sleep(1000);
            }

            //distance sensors
            if(gamepad1.b){
                telemetry.addData("DISTANCE: ", frontDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
                sleep(1000);
            }

            //webcam
            if(gamepad1.x){
                webcamTester.runOpMode();
            }

            //wobble servo
            if(gamepad1.y){
                double up = 0.8;
                double down = 0.2;
                wobbleServo.setPosition(up);
                wobbleServo.setPosition(down);
                telemetry.addData("WOBBLE SERVO POSITION: ", wobbleServo.getPosition());
                telemetry.update();
                sleep(1000);
            }

            //drivetrain motors
            if(gamepad1.dpad_up){
                frontRightMotor.setTargetPosition(1000);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setPower(0.5);
                telemetry.addData("FRONT RIGHT", frontRightMotor.getCurrentPosition());
                telemetry.update();
                while (frontRightMotor.isBusy()) {/*do nothing*/}
                frontRightMotor.setPower(0);
            }
            if(gamepad1.dpad_left){
                frontLeftMotor.setTargetPosition(1000);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftMotor.setPower(0.5);
                telemetry.addData("FRONT LEFT", frontLeftMotor.getCurrentPosition());
                telemetry.update();
                while (frontLeftMotor.isBusy()) {/*do nothing*/}
                frontLeftMotor.setPower(0);
            }
            if(gamepad1.dpad_down){
                backLeftMotor.setTargetPosition(1000);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setPower(0.5);
                telemetry.addData("BACK LEFT", backLeftMotor.getCurrentPosition());
                telemetry.update();
                while (backLeftMotor.isBusy()) {/*do nothing*/}
                backLeftMotor.setPower(0);
            }
            if(gamepad1.dpad_right){
                backRightMotor.setTargetPosition(1000);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setPower(0.5);
                telemetry.addData("BACK RIGHT", backRightMotor.getCurrentPosition());
                telemetry.update();
                while (backRightMotor.isBusy()) {/*do nothing*/}
                backRightMotor.setPower(0);
            }

            //shooting motors
            if(gamepad1.right_bumper){
                shootingMotor.setTargetPosition(1000);
                shootingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shootingMotor.setPower(0.5);
                telemetry.addData("SHOOTING MOTOR", shootingMotor.getCurrentPosition());
                telemetry.update();
                while (shootingMotor.isBusy()) {/*do nothing*/}
                shootingMotor.setPower(0);
            }
        }
    }
}