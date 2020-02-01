package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Navigation Test", group = "autonomous")
public class NavigationTester extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    BNO055IMU imu;

    Servo sideClampServo;
    Servo sideArmServo;
    public void initialize() {
        //Configuration of the motors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        sideClampServo = hardwareMap.get(Servo.class, "blockClamper");

        sideArmServo = hardwareMap.get(Servo.class, "pivotGrabber");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        /*frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sideClampServo.setPosition(0.5);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        sideArmServo.setPosition(0.4);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }

        telemetry.addData("imu calibration status: ", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            NavigationHelper navigateTest = new NavigationHelper();

            //navigateTest.navigate(31, Constants12907.Direction.RIGHT, 0, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

            sideArmServo.setPosition(0.8);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            sideClampServo.setPosition(1);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            sideArmServo.setPosition(0.4);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }



            //navigateTest.navigate(5, Constants12907.Direction.RIGHT, 0, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

           /* navigateTest.navigate(23, Constants12907.Direction.STRAIGHT,0,0.65, backLeft, backRight, frontRight, frontLeft, imu, telemetry);
            navigateTest.navigate(11, Constants12907.Direction.STRAIGHT,0,0.4, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

            navigateTest.navigate(-41, Constants12907.Direction.STRAIGHT,0,-0.65, backLeft, backRight, frontRight, frontLeft, imu, telemetry);
            navigateTest.navigate(-21, Constants12907.Direction.STRAIGHT,0,-0.4, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

            //navigateTest.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

            navigateTest.navigate(66, Constants12907.Direction.STRAIGHT,0,0.75, backLeft, backRight, frontRight, frontLeft, imu, telemetry);
            navigateTest.navigate(-20, Constants12907.Direction.STRAIGHT,0,-0.6, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

            navigateTest.navigate(3, Constants12907.Direction.STRAIGHT,0,0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry);
*/
                   /* navigateTest.navigate(23, Constants12907.Direction.STRAIGHT,0,0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry);
                    navigateTest.navigate(11, Constants12907.Direction.STRAIGHT,0,0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

                    navigateTest.navigate(-41, Constants12907.Direction.STRAIGHT,0,-0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry);
                    navigateTest.navigate(-21, Constants12907.Direction.STRAIGHT,0,-0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

                    //navigateTest.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

                    navigateTest.navigate(66, Constants12907.Direction.STRAIGHT,0,0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry);
                    navigateTest.navigate(-20, Constants12907.Direction.STRAIGHT,0,-0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

                    navigateTest.navigate(3, Constants12907.Direction.STRAIGHT,0,0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry);
                    */

            //backLeft.setPower(0);
        }
    }
}