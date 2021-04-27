package org.firstinspires.ftc.UltimateGoalTeamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.UltimateGoalTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.Constants2020;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.DetectionHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.opmode.UltimateAuto;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

        resetTheImu(variableMap);
        Orientation beginning = imu.getAngularOrientation();
        float correct = beginning.firstAngle;


        //move to tgt zone A, B, or C
        if (position.equals(Constants2020.TargetZone.ALPHA)) {
            telemetry.addLine("RED - WALL - ALPHA");
            telemetry.update();
            if(isWall){
                //CHANGE NEEDED (speed and distance): -38 to -28 and 0.45 to 0.9
                navigater.navigate(alphaDist - 28, Constants2020.Direction.STRAIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if(!isWall){
                navigater.navigate((alphaDist - 12.75) + 15, Constants2020.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }

        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            telemetry.addLine("RED - WALL - BETA");
            telemetry.update();
            //red wall beta
            if(isWall){
                //CHANGE NEEDED (speed and distance)
                navigater.navigate(betaDistance + 2.25 - 40 /*- 10 - 5 - 5*/, Constants2020.Direction.STRAIGHT, 0, 0.5/*0.9*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
          //red not wall beta
            } else if(!isWall){
                navigater.navigate(betaDistance + 2.25 - 20, Constants2020.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }

        } else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            telemetry.addLine("RED - WALL - CHARLIE");
            telemetry.update();
            //red wall charlie
            if(isWall){
                //CHANGE NEEDED (speed and distance)
                //-45
                navigater.navigate(charlieDistance - 35, Constants2020.Direction.STRAIGHT, 0, 0.45/*0.9*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
                //dhruvs idea on how to stop, maybe implement later?
                /*backLeft.setPower(-0.01);
                backRight.setPower(-0.01);
                frontLeft.setPower(-0.01);
                frontRight.setPower(-0.01);
                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                 */
           //red not wall charlie
            } else if(!isWall){
                navigater.navigate(charlieDistance, Constants2020.Direction.STRAIGHT, 0, 0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }

        //attempt at correcting left because of extreme veering
        //feel free to comment out at any time
        Orientation boo = imu.getAngularOrientation();
        float needsHelp = boo.firstAngle;

        float diff = needsHelp-correct;
        telemetry.addData("imu is off by" , diff);
        telemetry.update();
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        if(diff>10){
            navigater.navigate(0, Constants2020.Direction.TURN, -diff, 0.5/*0.25*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }

        else if(diff<-10){
            navigater.navigate(0, Constants2020.Direction.TURN, -diff, 0.5/*0.25*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }



        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        resetTheImu(variableMap);

        //adjust position at tgt zone  B  to drop wobble goal:
        if (position.equals(Constants2020.TargetZone.BETA)) {
            if (isWall && !isBlue) {
                //CHANGE NEEDED (speed and distance)
                navigater.navigate(12, Constants2020.Direction.LEFT, 0, 0.3/*0.5*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else if (!isWall && !isBlue){
                //red not wall beta
                navigater.navigate(0, Constants2020.Direction.TURN, 82.5, 0.5/*0.25*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }
        resetTheImu(variableMap);
    }

    public void dropWobbleGoal(HashMap<String, Object> variableMap){
        final int WAIT = 500; //1000
        Telemetry telemetry = (Telemetry) variableMap.get(Constants2020.TELEMETRY);
        DcMotor wobbleHingeServo = (DcMotor) variableMap.get(Constants2020.HINGE_SERVO);
        wobbleHingeServo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo wobbleClampServo = (Servo) variableMap.get(Constants2020.CLAMP_SERVO);
        double clampServoOut = 0.2;
        double hingeServoOut = 0.05;
        double clampServoMid = 0.45;
        double hingeServoIn = 0.95;
        telemetry.addLine("Inside dropWobbleGoal");
        telemetry.update();

        //put arm out
        wobbleHingeServo.setPower(0.45);
        try {
            sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        wobbleHingeServo.setPower(0);
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


        try {
            sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        wobbleHingeServo.setPower(-0.5);
        try {
            sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        wobbleHingeServo.setPower(0);

    }

}
