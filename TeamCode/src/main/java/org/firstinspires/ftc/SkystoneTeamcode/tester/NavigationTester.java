package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        if(opModeIsActive()) {
            NavigationHelper navigateTest = new NavigationHelper();
           navigateTest.navigate(100, Constants12907.Direction.STRAIGHT,0,0.45, backLeft, backRight, frontRight, frontLeft, imu, telemetry);
           // navigateTest.navigate(-40, Constants12907.Direction.STRAIGHT, 0, -0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry);

            /*telemetry.addData("Path1", "Running to %7d :%7d", 0,0);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
                   //frontLeft.getCurrentPosition();
                    //frontRight.getCurrentPosition();
            telemetry.update();*/
        }


    }


}
