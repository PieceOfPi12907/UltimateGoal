package org.firstinspires.ftc.UltimateGoalTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "ULTIMATE AUTO", group = "autonomous")
public class UltimateAuto extends LinearOpMode {

    Boolean isBlue = false;
    Boolean isWall = false;
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    OpenCvCamera webcam;

    public void initialize() {

        isBlue = false;
        isWall = false;

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

        initialize();

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
        }
    }
}
