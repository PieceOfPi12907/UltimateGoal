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
        if (isBlue && isWall) {
            navigater.navigate(6, Constants12907.Direction.LEFT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else if (!isBlue && isWall) {
            navigater.navigate(6, Constants12907.Direction.RIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }

        resetTheImu(variableMap);

        //move to tgt zone A, B, or C
        if (position.equals(Constants2020.TargetZone.ALPHA)) {
            telemetry.addLine("RED - WALL - ALPHA");
            telemetry.update();
            navigater.navigate(alphaDist - 12.75, Constants12907.Direction.STRAIGHT, 0, 0.4, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            telemetry.addLine("RED - WALL - BETA");
            telemetry.update();
            navigater.navigate(betaDistance + 2.25, Constants12907.Direction.STRAIGHT, 0, 0.4, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            telemetry.addLine("RED - WALL - CHARLIE");
            telemetry.update();
            navigater.navigate(charlieDistance - 5, Constants12907.Direction.STRAIGHT, 0, 0.4, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }

        resetTheImu(variableMap);

        //adjust position at tgt zone A, B, or C to drop wobble goal:
        /*if (position.equals(Constants2020.TargetZone.ALPHA)) {
            if (isBlue && !isWall) {
                //Blue Not Wall
                navigater.navigate(0, Constants12907.Direction.TURN, 180, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (!isBlue && !isWall) {
                //Red Not Wall
               navigater.navigate(20, Constants12907.Direction.STRAIGHT, 0, 0.4, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
               navigater.navigate(24, Constants12907.Direction.RIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (isWall) {
                //Red and Blue Wall
                navigater.navigate(0, Constants12907.Direction.TURN, 90, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            if (isWall) {
                //for Red wall
                navigater.navigate(10, Constants12907.Direction.LEFT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }
        else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            if (isWall) {
                //red and blue wall
                navigater.navigate(0, Constants12907.Direction.TURN, -90, 0.25, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (!isWall) {
                //red not wall
                navigater.navigate(10, Constants12907.Direction.STRAIGHT, 0, 0.4, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
                navigater.navigate(24, Constants12907.Direction.RIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
            //drop wobble goal
        }*/
    }
}
