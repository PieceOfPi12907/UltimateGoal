package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Control Hub Servo Test", group = "autonomous")
public class ControlHubServoTest extends LinearOpMode {
    //BNO055IMU imu;
    Servo testServo;
    double position = 0.5;

    public void initialize() {
        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);*/

        testServo = hardwareMap.get(Servo.class, "testservo");
        telemetry.addLine("initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        /*while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }*/

        /*telemetry.addData("imu calibration status: ", imu.getCalibrationStatus().toString());
        telemetry.update();*/

        waitForStart();

        if(opModeIsActive()) {
            testServo.setPosition(position);

            sleep(500);

            telemetry.addData("moved to ", position);
            telemetry.update();

            sleep(1000);
        }
    }
}