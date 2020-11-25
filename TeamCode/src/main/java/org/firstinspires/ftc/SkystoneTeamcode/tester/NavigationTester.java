package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Moving Test", group = "autonomous")
public class NavigationTester extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    BNO055IMU imu;

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

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            navigateTest.navigate(60, Constants12907.Direction.LEFT,0,0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, false);

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            navigateTest.navigate(60, Constants12907.Direction.RIGHT,0,0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }


        }
    }
}