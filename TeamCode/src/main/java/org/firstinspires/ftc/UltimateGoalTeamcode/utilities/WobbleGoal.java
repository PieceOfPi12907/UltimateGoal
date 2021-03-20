package org.firstinspires.ftc.UltimateGoalTeamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.UltimateGoalTeamcode.helper.NavigationHelper;
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
        telemetry.addLine("Inside moveToTgtZone");
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
        double alphaDist = 65.75;
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
            //red wall alpha
            if(isWall){
                telemetry.addLine("about to move");
                telemetry.update();
                //with new wobble goal attachment, made -12.75 -30
                navigater.navigate(alphaDist - 30, Constants2020.Direction.STRAIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
           //red not wall alpha
            } else if(!isWall){
                navigater.navigate((alphaDist - 12.75) + 15, Constants2020.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }

        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            telemetry.addLine("RED - WALL - BETA");
            telemetry.update();
            //red wall beta
            if(isWall){
                navigater.navigate(betaDistance + 2.25 - 12, Constants2020.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
          //red not wall beta
            } else if(!isWall){
                navigater.navigate(betaDistance + 2.25 - 20, Constants2020.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }

        } else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            telemetry.addLine("RED - WALL - CHARLIE");
            telemetry.update();
            //red wall charlie
            if(isWall){
                navigater.navigate(charlieDistance - 5 - 5 - 5, Constants2020.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
           //red not wall charlie
            } else if(!isWall){
                navigater.navigate(charlieDistance, Constants2020.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }

        resetTheImu(variableMap);

        //adjust position at tgt zone A, B, or C to drop wobble goal:
        if (position.equals(Constants2020.TargetZone.ALPHA)) {
            if (isBlue && !isWall) {
                //blue not wall alpha
                navigater.navigate(0, Constants2020.Direction.TURN, 180, 0.5/*0.25*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (isWall) {
                //red and blue wall alpha
                navigater.navigate(0, Constants2020.Direction.TURN, 85, 0.25/*0.25*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }

        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            if (isWall && !isBlue) {
                //red wall beta
                navigater.navigate(0, Constants2020.Direction.TURN, 175, 0.5/*0.25*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                navigater.navigate(5, Constants2020.Direction.RIGHT, 0, 0.5/*0.6*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);

            } else if (!isWall && !isBlue){
                //red not wall beta
                navigater.navigate(0, Constants2020.Direction.TURN, 82.5, 0.5/*0.25*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }
        else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            if (isWall) {
                //red and blue wall charlie
                navigater.navigate(0, Constants2020.Direction.TURN, 85, 0.5/*0.25*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }

        resetTheImu(variableMap);

    }

    public void dropWobbleGoal(HashMap<String, Object> variableMap){
        final int WAIT = 500; //1000
        Telemetry telemetry = (Telemetry) variableMap.get(Constants2020.TELEMETRY);
        Servo wobbleHingeServo = (Servo) variableMap.get(Constants2020.HINGE_SERVO);
        Servo wobbleClampServo = (Servo) variableMap.get(Constants2020.CLAMP_SERVO);
        double clampServoOut = 0.2;
        double hingeServoOut = 0.05;
        double clampServoMid = 0.45;
        double hingeServoIn = 0.95;
        telemetry.addLine("Inside dropWobbleGoal");
        telemetry.update();

        //put arm out
        wobbleHingeServo.setPosition(hingeServoOut);
        try {
            sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //let it go
        wobbleClampServo.setPosition(clampServoOut);
        try {
            sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //grab it
        wobbleClampServo.setPosition(clampServoMid);
        try {
            sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        wobbleHingeServo.setPosition(hingeServoIn);
        try {
            sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
