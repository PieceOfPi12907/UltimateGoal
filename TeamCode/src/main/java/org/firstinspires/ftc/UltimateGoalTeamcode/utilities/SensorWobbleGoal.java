package org.firstinspires.ftc.UltimateGoalTeamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.UltimateGoalTeamcode.helper.SensorHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.Constants2020;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class SensorWobbleGoal {

    //WOBBLE GOAL: RIGHT SIDE OF ROBOT

    NavigationHelper navigater = new NavigationHelper();
    SensorHelper sensorhelp = new SensorHelper();


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
        ColorSensor frontColor = (ColorSensor) variableMap.get(Constants2020.FRONT_COLOR);
        Constants2020.TargetZone position = (Constants2020.TargetZone) variableMap.get(Constants2020.POSITION);
        //distances to tgt zones specified in game manual:
        double alphaDist = 70.75;
        double betaDistance = 94.625;
        double charlieDistance = 118.372;
        double launchLineDist = 80;

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
            telemetry.addLine("RED ALPHA");
            telemetry.update();
            if(isWall){
                sensorhelp.moveUntilColor(0.5, (alphaDist - 12.75), true, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true, frontColor);
            } else if(!isWall){
                sensorhelp.moveUntilColor(0.5, launchLineDist, false, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true, frontColor);
                navigater.navigate(10, Constants2020.Direction.STRAIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
                //when white detected, travel "x" more inches
            }
        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            telemetry.addLine("RED BETA");
            telemetry.update();
            if(isWall){
                sensorhelp.moveUntilColor(0.5, launchLineDist, false, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true, frontColor);
                navigater.navigate(30, Constants2020.Direction.STRAIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
                //when white detected --> move "x" inches (OR: move until red detected for full sensor usage)
            } else if(!isWall){
                sensorhelp.moveUntilColor(0.5, (betaDistance + 2.25 - 20), true, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true, frontColor);
                //power to motors --> when red detected --> stop
            }
        } else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            telemetry.addLine("RED CHARLIE");
            telemetry.update();
            if(isWall){
                sensorhelp.moveUntilColor(0.5, launchLineDist, false, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true, frontColor);
                navigater.navigate(30, Constants2020.Direction.STRAIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
                //detect white --> move "x" inches --> stop
            } else if(!isWall){
                sensorhelp.moveUntilColor(0.5, charlieDistance, true, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true, frontColor);
                navigater.navigate(30, Constants2020.Direction.STRAIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
                //when red detected --> travel "x" more inches
            }
        }

        resetTheImu(variableMap);

        //adjust position at tgt zone A, B, or C to drop wobble goal:
        if (position.equals(Constants2020.TargetZone.ALPHA)) {
            if (isBlue && !isWall) {
                //blue not wall
                navigater.navigate(0, Constants2020.Direction.TURN, 180, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (isWall) {
                //red and blue wall
                navigater.navigate(0, Constants2020.Direction.TURN, 85, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            if (isWall && !isBlue) {
                //red wall
                navigater.navigate(0, Constants2020.Direction.TURN, 175, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (!isWall && !isBlue){
                //red not wall
                navigater.navigate(0, Constants2020.Direction.TURN, 82.5, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }
        else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            if (isWall) {
                //red and blue wall
                navigater.navigate(0, Constants2020.Direction.TURN, 85, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }
        resetTheImu(variableMap);
    }
    //drop wobble goal
}
