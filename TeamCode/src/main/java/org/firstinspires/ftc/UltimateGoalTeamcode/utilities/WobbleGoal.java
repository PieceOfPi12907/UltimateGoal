package org.firstinspires.ftc.UltimateGoalTeamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.Constants2020;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.DetectionHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.opmode.UltimateAuto;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import static java.lang.Thread.sleep;

public class WobbleGoal {

    //WOBBLE GOAL: RIGHT SIDE OF ROBOT

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

    public void moveToTgtZone(HashMap<String, Object> variableMap) {
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
        double betaDistance = 94.625;
        double charlieDistance = 118.372;

        //If isWall is true, strafe right (red) or left (blue) to the wall
        /*if (isBlue && isWall) {
            navigater.navigate(6, Constants12907.Direction.LEFT, 0, 0.6, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else if (!isBlue && isWall) {
            //changed six inches to three because it was going way to much right
            navigater.navigate(3.5, Constants12907.Direction.RIGHT, 0, 0.6, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }*/

        resetTheImu(variableMap);

        //move to tgt zone A, B, or C
        if (position.equals(Constants2020.TargetZone.ALPHA)) {
            telemetry.addLine("RED - WALL - ALPHA");
            telemetry.update();
            if(isWall){
                navigater.navigate(alphaDist - 12.75, Constants12907.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if(!isWall){
                //added this distance for not wall on 1/23
                navigater.navigate((alphaDist - 12.75) + 15, Constants12907.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            telemetry.addLine("RED - WALL - BETA");
            telemetry.update();
            if(isWall){
                //changed distance minus 12 inches on 1/21
                navigater.navigate(betaDistance + 2.25 - 12, Constants12907.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if(!isWall){
                //moved back this distance for not wall on 1/23
                navigater.navigate(betaDistance + 2.25 - 20, Constants12907.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            telemetry.addLine("RED - WALL - CHARLIE");
            telemetry.update();
            if(isWall){
                //changed distance minus 5 inches on 1/21
                navigater.navigate(charlieDistance - 5 - 5 - 5, Constants12907.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if(!isWall){
                navigater.navigate(charlieDistance, Constants12907.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }


        resetTheImu(variableMap);

        //adjust position at tgt zone A, B, or C to drop wobble goal:
        if (position.equals(Constants2020.TargetZone.ALPHA)) {
            if (isBlue && !isWall) {
                //Blue Not Wall
                navigater.navigate(0, Constants12907.Direction.TURN, 180, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (isWall) {
                //Red and Blue Wall
                navigater.navigate(0, Constants12907.Direction.TURN, 85, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            if (isWall && !isBlue) {
                //for Red wall
                //changed strafe from 10 to 18 on 1/21
                //navigater.navigate(18, Constants12907.Direction.LEFT, 0, 0.6, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
                //resetTheImu(variableMap);
                navigater.navigate(0, Constants12907.Direction.TURN, 175, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (!isWall && !isBlue){
                navigater.navigate(0, Constants12907.Direction.TURN, 82.5, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }
        else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            if (isWall) {
                //red and blue wall
                //changed turn from -90 to 90 on 1/21
                navigater.navigate(0, Constants12907.Direction.TURN, 85, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (!isWall) {
                //red not wall
                //navigater.navigate(24, Constants12907.Direction.RIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }
        resetTheImu(variableMap);
    }
    //drop wobble goal
}
