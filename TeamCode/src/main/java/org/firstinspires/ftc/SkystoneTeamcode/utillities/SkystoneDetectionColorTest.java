package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.helper.SensorHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Skystone Detection Color Test", group = "autonomous")
public class SkystoneDetectionColorTest extends LinearOpMode{

    BNO055IMU imu;
    SkystoneDetection skystoneDetection;
    NavigationHelper navigationHelper;
    ColorSensor colorRight;
    DistanceSensor distanceRight;
    ColorSensor colorLeft;
    DistanceSensor distanceLeft;

    public void initialize() {
        //Initializing color and distance sensors
        colorRight=hardwareMap.get(ColorSensor.class,"sensor_color_distance_right");
        distanceRight=hardwareMap.get(DistanceSensor.class,"sensor_color_distance_right");
        colorLeft=hardwareMap.get(ColorSensor.class,"sensor_color_distance_left");
        distanceLeft=hardwareMap.get(DistanceSensor.class,"sensor_color_distance_left");
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

    navigationHelper = new NavigationHelper();
    skystoneDetection = new SkystoneDetection();

    waitForStart();

    if (opModeIsActive()) {
        moveToSkystoneOuterBlue(imu,telemetry,colorRight,colorLeft,distanceRight,distanceLeft);
    }
    }

    SensorHelper sensorHelper = new SensorHelper();

    public void moveToSkystoneOuterBlue(BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeft) {
        //Moving to the wall
        //pNavigate.navigate(-25, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //Strafing to the Skystone - sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
        // pNavigate.navigate(29.5, Constants12907.Direction.RIGHT, 0, 0.5, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        //Color Sensing Code

        boolean isLYellow = sensorHelper.isYellow(colorLeft);
        boolean isRYellow = sensorHelper.isYellow(colorRight);
        int getBlackBlock = sensorHelper.getBlackBlock(colorLeft,colorRight);
        pTelemetry.addData("Red Right:  ", colorRight.red());
        pTelemetry.addData("Green Right: ", colorRight.green());
        pTelemetry.addData("Blue Right: ", colorRight.blue());


        pTelemetry.addData("Red Left:  ", colorLeft.red());
        pTelemetry.addData("Green Left: ", colorLeft.green());
        pTelemetry.addData("Blue Left: ", colorLeft.blue());

        pTelemetry.update();

        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if (isRYellow && isLYellow) {
            pTelemetry.addData("position: ", 3);
        } else if (isLYellow && (isRYellow == false)) {
            pTelemetry.addData("position: ", 2);
        } else {
            pTelemetry.addData("position: ", 1);
        }
        pTelemetry.update();

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pTelemetry.addData("position from getBlackBlock",getBlackBlock);
        pTelemetry.update();
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}
