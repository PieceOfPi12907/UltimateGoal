package org.firstinspires.ftc.UltimateGoalTeamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.Constants2020;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class SensorShootingRings {

    NavigationHelper navigater = new NavigationHelper();

    public void resetTheImu(HashMap<String, Object> variableMap) {
        BNO055IMU imu = (BNO055IMU) variableMap.get(Constants2020.IMU);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);
    }

    public void moveToLaunchLine(HashMap<String, Object> variableMap) {
        Telemetry telemetry = (Telemetry) variableMap.get(Constants2020.TELEMETRY);
        telemetry.addLine("Inside method in Wobble Goal Class");
        telemetry.update();
        boolean isWall = (boolean) (variableMap.get(Constants2020.WALL_FLAG));
        boolean isBlue = (boolean) (variableMap.get(Constants2020.BLUE_FLAG));
        BNO055IMU imu = (BNO055IMU) variableMap.get(Constants2020.IMU);
        DcMotor backLeft = (DcMotor) variableMap.get(Constants2020.BACK_LEFT_MOTOR);
        DcMotor backRight = (DcMotor) variableMap.get(Constants2020.BACK_RIGHT_MOTOR);
        DcMotor frontLeft = (DcMotor) variableMap.get(Constants2020.FRONT_LEFT_MOTOR);
        DcMotor frontRight = (DcMotor) variableMap.get(Constants2020.FRONT_RIGHT_MOTOR);
        Constants2020.TargetZone position = (Constants2020.TargetZone) variableMap.get(Constants2020.POSITION);
        //distances to tgt zones specified in game manual:
        double alphaDist = 70.75;
        double betaDist = 94.625;
        double charlieDist = 118.372;

        //reverse adjustments at tgt zone A, B, or C
        if (position.equals(Constants2020.TargetZone.ALPHA)) {
            if (isBlue && !isWall) {
                //blue not wall
                navigater.navigate(0, Constants12907.Direction.TURN, 180, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (isWall) {
                //red and blue wall
                navigater.navigate(0, Constants12907.Direction.TURN, -85, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            if (isWall) {
                //red wall
                navigater.navigate(0, Constants12907.Direction.TURN, -175, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (!isWall){
                //red not wall
                navigater.navigate(0, Constants12907.Direction.TURN, -82.5, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }
        else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            if (isWall) {
                //red and blue wall
                navigater.navigate(0, Constants12907.Direction.TURN, -85, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }

        resetTheImu(variableMap);

        //back up behind the launch line
        if(position.equals(Constants2020.TargetZone.ALPHA)){
            if(isWall){
                //already behind launch line
            } else if(!isWall){
                navigater.navigate(-15, Constants12907.Direction.STRAIGHT, 0, -0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            if(isWall){
                navigater.navigate(-((betaDist/3)-3), Constants12907.Direction.STRAIGHT, 0, -0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if(!isWall){
                navigater.navigate(-((betaDist/3)-3-11), Constants12907.Direction.STRAIGHT, 0, -0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            if(isWall){
                navigater.navigate(-((charlieDist/2)-3.186 - 5 - 5), Constants12907.Direction.STRAIGHT, 0, -0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (!isWall){
                navigater.navigate(-((charlieDist/2)), Constants12907.Direction.STRAIGHT, 0, -0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }
        //shoot rings

        resetTheImu(variableMap);

        //parking (over the launch line)
        navigater.navigate(9, Constants12907.Direction.STRAIGHT, 0, 0.4, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
    }
}
