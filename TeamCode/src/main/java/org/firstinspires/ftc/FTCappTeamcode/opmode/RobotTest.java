package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helper.SensorHelper;



/**
This program tests robot servos, motors, and sensors (they are all assigned to buttons)
 */
@TeleOp(name = "Robot Test", group = "teleop")
public class RobotTest extends LinearOpMode {

    //Add telemetries where needed!

    //Naming the motors
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor lift;
    DcMotor slide;
    DcMotor sweeper;
    Servo latch;
    Servo sweeperDump;
    Servo sweeperDumpRight;
    Servo sweeperDumpLeft;
    Servo mineralDump;
    Servo rightKnocker;
    Servo leftKnocker;
    Servo rightArm;
    Servo leftArm;
    ColorSensor knockerColorSensor;
    ColorSensor markerColorSensor;
    ColorSensor middleColorSensor;
    DistanceSensor distanceSensor;

    public  void initialize () {

//servos & motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        lift = hardwareMap.get(DcMotor.class, "lift");
        slide = hardwareMap.get(DcMotor.class, "slide");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");
        latch = hardwareMap.get(Servo.class, "latch");
        sweeperDump = hardwareMap.get(Servo.class, "sweeperDump");
        sweeperDumpRight = hardwareMap.get(Servo.class, "sweeperDumpRight");
        sweeperDumpLeft = hardwareMap.get(Servo.class, "sweeperDumpLeft");
        mineralDump = hardwareMap.get(Servo.class, "mineralDump");
        rightKnocker = hardwareMap.get(Servo.class, "rightKnocker");
        leftKnocker = hardwareMap.get(Servo.class, "leftKnocker");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
//sensors
        markerColorSensor = hardwareMap.get(ColorSensor.class, "markerColor");
        middleColorSensor = hardwareMap.get(ColorSensor.class, "middleColor");
        knockerColorSensor = hardwareMap.get(ColorSensor.class, "knockerColor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "middleColor");

        //Setting the direction of the motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        sweeper.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        double powerLeft = 1;
        double powerRight = 1;
        double powerLift = 1;
        double powerSlide = 1;
        double powerSweep = 1;

        SensorHelper sensorHelper = new SensorHelper();

        waitForStart();

        while (opModeIsActive()) {
        /*
        Gamepad:
            4 drivetrain motors  ---> (1) left & right joysticks
            Slide motor ----> (1) right & left triggers
            Right knocker servo ------> (1) a & b
            Right arm servo ------------> (1) x & y
            Left knocker servo ---------> (1) left & up d-pad
            Left arm servo --------------------> (1) down & right d-pad
            Sweeper motor -----------------> (2) right joystick
            Linear Actuator motor -------> (2) left joystick
            Hook servo ----------> (2) left & right d-pad
            Sweeper dump servo -------> (2) a & b
            Mineral dump servo ---------> (2) x & y

            knocker Color Sensor ------> (1) right bumper
            marker Color Sensor -------> (1) left bumper
            middle Color Sensor -------> (2) right bumper
            distance Sensor -----------> (2) left bumper
         */

            //ADD TELEMETRIES where needed

            //Knocker Color Sensor
            if (gamepad1.right_bumper) {
                boolean whiteLeft = sensorHelper.isWhite(knockerColorSensor, telemetry);
                telemetry.addData("(false = gold) Left Color: ", whiteLeft);
                telemetry.update();
            }

            //Middle Color Sensor
            if (gamepad1.left_bumper) {
                boolean whiteMiddle = sensorHelper.isWhite(middleColorSensor, telemetry);
                telemetry.addData("(false = gold) Middle Color: ", whiteMiddle);
                telemetry.update();
            }

            //Marker Color Sensor
            if (gamepad2.right_bumper) {
                boolean turnRight = sensorHelper.isWhite(markerColorSensor, telemetry);
                telemetry.addData("True (white) or False (black): ", turnRight);
                telemetry.update();
            }

            //Distance Sensor
            if (gamepad2.right_bumper) {
                double distanceMeasured = (distanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.addData("Distance from object: ", distanceMeasured);
                telemetry.update();
            }

            //Right Knocker Servo
            if (gamepad1.a) {
                //in
                rightKnocker.setPosition(1);
            }

            if (gamepad1.b) {
                //out
                rightKnocker.setPosition(0);
            }

            //Right Arm Servo
            if (gamepad1.x) {
                //down
                rightArm.setPosition(0);
            }
            if (gamepad1.y) {
                //up
                rightArm.setPosition(1);
            }

            //Left Knocker Servo
            if (gamepad1.dpad_left) {
                //in
                leftKnocker.setPosition(0);
            }
            if (gamepad1.dpad_up) {
                //out
                leftKnocker.setPosition(1);
            }

            //Left Arm Servo
            if (gamepad1.dpad_down) {
                //down
                leftArm.setPosition(1);
            }
            if (gamepad1.dpad_right) {
                //up
                leftArm.setPosition(0);
            }

            //joysticks to power the drivetrain motors
            powerLeft = -gamepad1.right_stick_y;
            frontLeft.setPower(powerLeft);
            backLeft.setPower(powerLeft);

            powerRight = -gamepad1.left_stick_y;
            frontRight.setPower(powerRight);
            backRight.setPower(powerRight);

            //using triggers to put the slide out and in
            powerSlide = gamepad1.right_trigger;
            slide.setPower(powerSlide * 0.9);

            powerSlide = gamepad1.left_trigger;
            slide.setPower(-(powerSlide * 0.9));

            //right joystick for the sweeper motor
            powerSweep = gamepad2.right_stick_y;
            sweeper.setPower(powerSweep);

            //lift linear actuator up and down for latching
            powerLift = -gamepad2.left_stick_y;
            lift.setPower(powerLift);

            //hook - latching servo
            if (gamepad2.dpad_left) {
                latch.setPosition(0.8);
            }
            if (gamepad2.dpad_right) {
                latch.setPosition(0.2);
            }

            //a & b for sweeper dump servo
        /*if (gamepad2.a){
            sweeperDump.setPosition(0.27);
        }
        if(gamepad2.b){
            sweeperDump.setPosition(0.8);
        }*/

            if (gamepad2.a) {
                sweeperDumpRight.setPosition(0.23);
                sweeperDumpLeft.setPosition(0.69);
            }
            if (gamepad2.b) {
                sweeperDumpRight.setPosition(0.7);
                sweeperDumpLeft.setPosition(0.17);
            }

            //x & y for mineral dump servo
            if (gamepad2.x) {
                mineralDump.setPosition(0.8);
            }
            if (gamepad2.y) {
                mineralDump.setPosition(0.2);
            }
        }
    }
}
