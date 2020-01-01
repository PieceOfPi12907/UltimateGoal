/*package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helper.MotorHelper;

@Autonomous(name = "AutonomousTest_Encoders", group = "autonomous")
public class AutoInnerOneBlockRepoBlue extends LinearOpMode {
    //Naming the motors
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    MotorHelper motorHelper;

    public void initialize() {
        //Configuration of the motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        //setting the directions of the motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        motorHelper = new MotorHelper();
        double powerRight = 0.25;
        double powerLeft = 0.25;
        double targetPositionLeft = 12;
        double targetPositionRight = 12;
        double timeoutS = 5;
        if (opModeIsActive()) {
            motorHelper.movingWithEncoders(frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor,
                    powerRight, powerLeft,
                    targetPositionRight, targetPositionLeft,
                    timeoutS, telemetry);
        }
    }
}
*/

package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Parking;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Repositioning;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//import java.util.Scanner;


//Adding Source Code to GitHub
@Disabled
@Autonomous(name = "Auto: PARKING", group = "autonomous")
public class AutoParking extends LinearOpMode {

    long startDelay = 0;

    //Naming the motors
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    BNO055IMU imu;
    SkystoneDetection skystoneDetection;
    NavigationHelper navigationHelper;
    ColorSensor colorRight;
    DistanceSensor distanceRight;
    ColorSensor colorLeft;
    DistanceSensor distanceLeft;
    DistanceSensor quarryDistance;
    WebcamName webcam;
    Servo pivotGrabber;
    Servo blockClamper;
    Servo repositioningRight;
    Servo repositioningLeft;


    public void initialize() {

        /*Scanner scanner = new Scanner(System.in);

        isBlue = scanner.nextBoolean();
        isOuter = scanner.nextBoolean();
        isPos2 = scanner.nextBoolean();

         */

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
        colorRight=hardwareMap.get(ColorSensor.class,"sensor_color_distance_right");
        distanceRight=hardwareMap.get(DistanceSensor.class,"sensor_color_distance_right");
        colorLeft=hardwareMap.get(ColorSensor.class,"sensor_color_distance_left");
        distanceLeft=hardwareMap.get(DistanceSensor.class,"sensor_color_distance_left");
        quarryDistance=hardwareMap.get(DistanceSensor.class,"sensor_distance_quarry");

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

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            initialize();
            navigationHelper = new NavigationHelper();
            skystoneDetection = new SkystoneDetection();
            Repositioning repositioning = new Repositioning();
            Parking parking = new Parking();

            while(!isStopRequested() && !imu.isGyroCalibrated()){
                sleep(50);
                idle();
            }

            telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
            telemetry.update();

            waitForStart();

            if (opModeIsActive()) {

                try {
                    Thread.sleep(startDelay);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //building zone --> park
                navigationHelper.navigate(-24, Constants12907.Direction.STRAIGHT,0,-0.5, backLeft, backRight, frontRight,frontLeft,imu,telemetry);

                //loading zone --> park
                //navigationHelper.navigate(24, Constants12907.Direction.STRAIGHT,0,0.5,backLeft, backRight, frontRight,frontLeft,imu,telemetry);


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

    }
}