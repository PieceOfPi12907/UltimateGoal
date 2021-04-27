package org.firstinspires.ftc.UltimateGoalTeamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.UltimateGoalTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.SensorHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.Constants2020;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.UltimateGoalTeamcode.tester.ColorSensorMeasure;

import java.util.HashMap;

public class ShootingRings {

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
        DcMotor shooter = (DcMotor) variableMap.get(Constants2020.SHOOTER);
        Servo shooterServo = (Servo) variableMap.get(Constants2020.SHOOTERSERVO);
        Constants2020.TargetZone position = (Constants2020.TargetZone) variableMap.get(Constants2020.POSITION);
        //distances to tgt zones specified in game manual:
        double alphaDist = 70.75;
        double betaDist = 94.625;
        double charlieDist = 118.372;

         final double SHOOTER_INTAKE_SERVO_OPEN = 0.5;
         final double SHOOTER_INTAKE_SERVO_CLOSE = 0.85;

        //back up behind the launch line
        if(position.equals(Constants2020.TargetZone.ALPHA)){
            if(isWall){
                //already behind launch line
            } else if(!isWall){
                navigater.navigate(-15, Constants2020.Direction.STRAIGHT, 0, -0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.BETA)) {
            if(isWall){
                //CHANGE NEEDED (speed and distance)
                navigater.navigate(-((betaDist/3)-20-5), Constants2020.Direction.STRAIGHT, 0, -0.45/*-0.75*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if(!isWall){
                navigater.navigate(-((betaDist/3)-3-11), Constants2020.Direction.STRAIGHT, 0, -0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        } else if (position.equals(Constants2020.TargetZone.CHARLIE)) {
            if(isWall){
                //CHANGE NEEDED (speed and distance)
                navigater.navigate(-((charlieDist/2)-43), Constants2020.Direction.STRAIGHT, 0, -0.45/*-0.75*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            } else if (!isWall){
                navigater.navigate(-((charlieDist/2)), Constants2020.Direction.STRAIGHT, 0, -0.9, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
            }
        }

        //strafe to be in front of the tower
        if(!position.equals(Constants2020.TargetZone.BETA)){
            //CHANGE NEEDED (speed and distance)
            //changed 12 to 8 on 4/17
            navigater.navigate(8, Constants2020.Direction.LEFT, 0, -0.45/*-0.75*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }
        resetTheImu(variableMap);
    }

    public void newRingShoot(HashMap<String, Object> variableMap){
        final double SHOOTER_INTAKE_SERVO_UP = 0.14;
        final int WAIT = 750; //300
        DcMotor shooter = (DcMotor) variableMap.get(Constants2020.SHOOTER);
        Servo shooterServo = (Servo) variableMap.get(Constants2020.SHOOTERSERVO);
        DcMotor backLeft = (DcMotor) variableMap.get(Constants2020.BACK_LEFT_MOTOR);
        DcMotor backRight = (DcMotor) variableMap.get(Constants2020.BACK_RIGHT_MOTOR);
        DcMotor frontLeft = (DcMotor) variableMap.get(Constants2020.FRONT_LEFT_MOTOR);
        DcMotor frontRight = (DcMotor) variableMap.get(Constants2020.FRONT_RIGHT_MOTOR);
        ColorSensor frontColor = (ColorSensor) variableMap.get(Constants2020.FRONT_COLOR);
        Telemetry telemetry = (Telemetry) variableMap.get(Constants2020.TELEMETRY);
        BNO055IMU imu = (BNO055IMU) variableMap.get(Constants2020.IMU);
        Constants2020.TargetZone position = (Constants2020.TargetZone) variableMap.get(Constants2020.POSITION);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setPower(0.9);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        double currentPos = 0.62;
        shooterServo.setPosition(currentPos);
        telemetry.addData("position", currentPos);
        telemetry.update();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        currentPos-=0.2;
        shooterServo.setPosition(currentPos);
        telemetry.addData("position",currentPos);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        currentPos-=0.15;
        shooterServo.setPosition(currentPos);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        currentPos-=0.2;
        shooterServo.setPosition(currentPos);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        currentPos = 0.57;
        shooterServo.setPosition(currentPos);
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        shooter.setPower(0);

        telemetry.addLine("PARK:");
        telemetry.update();

        //SENSOR PARKING
        //sensorhelp.moveUntilColor(0.5, 5, false, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true, frontColor);

        //NOW USED PARKING - COMMENT THIS OUT WHEN USING DO EXTRA RINGS
        navigater.navigate(5, Constants2020.Direction.STRAIGHT, 0, 0.75/*0.4*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
    }

    public void doExtraRings(HashMap<String, Object> variableMap) {
        final double SHOOTER_INTAKE_SERVO_UP = 0.14;
        final int WAIT = 750; //300
        DcMotor shooter = (DcMotor) variableMap.get(Constants2020.SHOOTER);
        Servo shooterServo = (Servo) variableMap.get(Constants2020.SHOOTERSERVO);
        DcMotor backLeft = (DcMotor) variableMap.get(Constants2020.BACK_LEFT_MOTOR);
        DcMotor backRight = (DcMotor) variableMap.get(Constants2020.BACK_RIGHT_MOTOR);
        DcMotor frontLeft = (DcMotor) variableMap.get(Constants2020.FRONT_LEFT_MOTOR);
        DcMotor frontRight = (DcMotor) variableMap.get(Constants2020.FRONT_RIGHT_MOTOR);
        ColorSensor frontColor = (ColorSensor) variableMap.get(Constants2020.FRONT_COLOR);
        Telemetry telemetry = (Telemetry) variableMap.get(Constants2020.TELEMETRY);
        BNO055IMU imu = (BNO055IMU) variableMap.get(Constants2020.IMU);
        boolean isWall = (boolean) (variableMap.get(Constants2020.WALL_FLAG));
        Constants2020.TargetZone position = (Constants2020.TargetZone) variableMap.get(Constants2020.POSITION);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        if (position.equals(Constants2020.TargetZone.BETA) || position.equals(Constants2020.TargetZone.CHARLIE)) {
            navigater.navigate(-10, Constants2020.Direction.STRAIGHT, 0, -0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);

            shooter.setPower(0.75);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            shooterServo.setPosition(0.42);
            try {
                Thread.sleep(WAIT);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            shooterServo.setPosition(0.22);
            try {
                Thread.sleep(WAIT);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            shooterServo.setPosition(0.02);
            try {
                Thread.sleep(WAIT);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            shooter.setPower(0);

            telemetry.addLine("PARK:");
            telemetry.update();

            //SENSOR PARKING
            //sensorhelp.moveUntilColor(0.5, 5, false, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true, frontColor);

            //NOW USED PARKING
            navigater.navigate(5, Constants2020.Direction.STRAIGHT, 0, 0.75/*0.4*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }
    }

    public void newPowerShoot(HashMap<String, Object> variableMap){
        final double SHOOTER_INTAKE_SERVO_UP = 0.14;
        final int WAIT = 300; //750
        DcMotor shooter = (DcMotor) variableMap.get(Constants2020.SHOOTER);
        Servo shooterServo = (Servo) variableMap.get(Constants2020.SHOOTERSERVO);
        DcMotor backLeft = (DcMotor) variableMap.get(Constants2020.BACK_LEFT_MOTOR);
        DcMotor backRight = (DcMotor) variableMap.get(Constants2020.BACK_RIGHT_MOTOR);
        DcMotor frontLeft = (DcMotor) variableMap.get(Constants2020.FRONT_LEFT_MOTOR);
        DcMotor frontRight = (DcMotor) variableMap.get(Constants2020.FRONT_RIGHT_MOTOR);
        ColorSensor frontColor = (ColorSensor) variableMap.get(Constants2020.FRONT_COLOR);
        Telemetry telemetry = (Telemetry) variableMap.get(Constants2020.TELEMETRY);
        BNO055IMU imu = (BNO055IMU) variableMap.get(Constants2020.IMU);
        Constants2020.TargetZone position = (Constants2020.TargetZone) variableMap.get(Constants2020.POSITION);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setPower(0.9);

        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //Turn to RIGHT power shot
        //navigater.navigate(0, Constants2020.Direction.TURN,20, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        /*try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
         */
        shooterServo.setPosition(0.42);
        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //Turn to MIDDLE power shot
        //navigater.navigate(0, Constants2020.Direction.TURN,20, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        navigater.navigate(5, Constants2020.Direction.LEFT,0, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);

        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(0.22);
        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //Turn to LEFT power shot
        //navigater.navigate(0, Constants2020.Direction.TURN,20, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        navigater.navigate(5, Constants2020.Direction.LEFT,0, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);

        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(0.02);
        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        shooter.setPower(0);

        telemetry.addLine("PARK:");
        telemetry.update();

        //SENSOR PARKING
        //sensorhelp.moveUntilColor(0.5, 5, false, backLeft, backRight, frontRight, frontLeft, telemetry, imu, true, frontColor);

        //NOW USED PARKING
        ////navigater.navigate(0, Constants2020.Direction.TURN,0 , 0.75/*0.5*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        if(position.equals(Constants2020.TargetZone.ALPHA)){
            //navigater.navigate(10, Constants2020.Direction.LEFT, 0, 0.75/*0.6*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }
        //navigater.navigate(5, Constants2020.Direction.STRAIGHT, 0, 0.75/*0.4*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);

    }
    public void powerShoot(HashMap<String, Object> variableMap){
        final double SHOOTER_INTAKE_SERVO_OPEN = 0.5;
        final double SHOOTER_INTAKE_SERVO_CLOSE = 0.85;
        final int WAIT = 500; //750
        DcMotor shooter = (DcMotor) variableMap.get(Constants2020.SHOOTER);
        Servo shooterServo = (Servo) variableMap.get(Constants2020.SHOOTERSERVO);
        DcMotor backLeft = (DcMotor) variableMap.get(Constants2020.BACK_LEFT_MOTOR);
        DcMotor backRight = (DcMotor) variableMap.get(Constants2020.BACK_RIGHT_MOTOR);
        DcMotor frontLeft = (DcMotor) variableMap.get(Constants2020.FRONT_LEFT_MOTOR);
        DcMotor frontRight = (DcMotor) variableMap.get(Constants2020.FRONT_RIGHT_MOTOR);
        Telemetry telemetry = (Telemetry) variableMap.get(Constants2020.TELEMETRY);
        BNO055IMU imu = (BNO055IMU) variableMap.get(Constants2020.IMU);
        Constants2020.TargetZone position = (Constants2020.TargetZone) variableMap.get(Constants2020.POSITION);


        //Turn to power shot leftmost
        //changed rotation from 14 to 13 to 11 to 5 WILL CHANGE BACK
        if(position.equals(Constants2020.TargetZone.ALPHA)){
            navigater.navigate(0, Constants2020.Direction.TURN,5 , 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else if (position.equals(Constants2020.TargetZone.BETA)){
            navigater.navigate(0, Constants2020.Direction.TURN,15, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else {
            navigater.navigate(0, Constants2020.Direction.TURN, 14, 0.75, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }
        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);
        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_OPEN);
        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(position.equals(Constants2020.TargetZone.ALPHA)){
            navigater.navigate(0, Constants2020.Direction.TURN,4 , 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else if (position.equals(Constants2020.TargetZone.BETA)){
            navigater.navigate(0, Constants2020.Direction.TURN, 14.5, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else {
            navigater.navigate(0, Constants2020.Direction.TURN, 13.5, 0.75, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }
        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);
        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_OPEN);
        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(position.equals(Constants2020.TargetZone.ALPHA)){
            navigater.navigate(0, Constants2020.Direction.TURN,3 , 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else if (position.equals(Constants2020.TargetZone.BETA)){
            navigater.navigate(0, Constants2020.Direction.TURN,14, 0.3, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        } else {
            navigater.navigate(0, Constants2020.Direction.TURN, 13, 0.75, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        }
        try {
            Thread.sleep(WAIT*2);
        } catch (InterruptedException e) {
            e.printStackTrace();}

        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);

        try {
            Thread.sleep(WAIT);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        /*shooterServo.setPosition(0.8);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/
        shooter.setPower(0);

        //new parking sequence

        //navigater.navigate(0, Constants2020.Direction.TURN,0 , 0.75/*0.5*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        navigater.navigate(10, Constants2020.Direction.LEFT, 0, 0.75/*0.6*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        navigater.navigate(5, Constants2020.Direction.STRAIGHT, 0, 0.75/*0.4*/, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
    }
    public void ringShoot(HashMap<String, Object> variableMap){
        final double SHOOTER_INTAKE_SERVO_OPEN = 0.5;
        final double SHOOTER_INTAKE_SERVO_CLOSE = 0.85;
        DcMotor shooter = (DcMotor) variableMap.get(Constants2020.SHOOTER);
        Servo shooterServo = (Servo) variableMap.get(Constants2020.SHOOTERSERVO);
        DcMotor backLeft = (DcMotor) variableMap.get(Constants2020.BACK_LEFT_MOTOR);
        DcMotor backRight = (DcMotor) variableMap.get(Constants2020.BACK_RIGHT_MOTOR);
        DcMotor frontLeft = (DcMotor) variableMap.get(Constants2020.FRONT_LEFT_MOTOR);
        DcMotor frontRight = (DcMotor) variableMap.get(Constants2020.FRONT_RIGHT_MOTOR);
        Telemetry telemetry = (Telemetry) variableMap.get(Constants2020.TELEMETRY);
        BNO055IMU imu = (BNO055IMU) variableMap.get(Constants2020.IMU);


        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_OPEN);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_OPEN);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooterServo.setPosition(0.8);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooter.setPower(0);
        navigater.navigate(10, Constants2020.Direction.LEFT, 0, 0.6, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
        navigater.navigate(5, Constants2020.Direction.STRAIGHT, 0, 0.4, backLeft, backRight, frontRight, frontLeft, imu, telemetry, true);
    }
}
