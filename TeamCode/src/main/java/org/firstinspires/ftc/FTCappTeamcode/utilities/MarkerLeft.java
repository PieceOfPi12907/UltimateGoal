package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.MotorHelper;

public class MarkerLeft {

    public void dropMarkerToDepot(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, MotorHelper motorHelper, Telemetry telemetry, BNO055IMU imu, Servo markerDropper, ColorSensor color) {
        //going back 8 inches
        double powerRight = -0.5;
        double powerLeft = -0.5;
        double targetPositionLeft = -5;
        double targetPositionRight = -5;
        double timeoutS = 3;
        motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight, targetPositionLeft, timeoutS, telemetry);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //turning towards wall
        float angle = 45;
        double turnPower = 0.5;
        motorHelper.turnWithEncoders(frontRight, frontLeft, backRight, backLeft, angle, turnPower, imu, telemetry);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //going forwards 30 inches to wall
        powerRight = 0.5;
        powerLeft = 0.5;
        targetPositionLeft = 30;
        targetPositionRight = 30;
        timeoutS = 6;
        motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight, targetPositionLeft, timeoutS, telemetry);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //detect if pie is white or black with color sensor
        /*SensorHelper sensorHelper = new SensorHelper();
        boolean turn = sensorHelper.isWhite(color, telemetry);
        telemetry.addData("turn: ",turn);
        telemetry.update();
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        /*if(turn){
            angle = -60;
            turnPower = 0.5;
            motorHelper.turnWithEncoders(frontRight, frontLeft, backRight, backLeft, angle, turnPower, imu);
            telemetry.addData("turn: ","Right");
            telemetry.update();
        }
        else{
            angle = 120;
            turnPower = 0.5;
            motorHelper.turnWithEncoders(frontRight, frontLeft, backRight, backLeft, angle, turnPower, imu);
            telemetry.addData("turn: ","Left");
            telemetry.update();
        }
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } */

        //turning towards depot
        angle = 20;
        turnPower = 0.5;
        motorHelper.turnWithEncoders(frontRight, frontLeft, backRight, backLeft, angle, turnPower, imu, telemetry);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //go forwards 4 feet
        targetPositionLeft = 45;
        targetPositionRight = 45;
        timeoutS = 6;
        motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight, targetPositionLeft, timeoutS, telemetry);

        //drop marker with servo movements
        final double MARKER_DROPPER_SERVO_POSITION = 0.2;
        //timeoutS = 5000;
        //motorHelper.markerDrop(markerDropper, MARKER_DROPPER_SERVO_POSITION, telemetry);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //go backwards 48 feet (INTO CRATER)
        //PARKING!!
        powerRight = -0.5;
        powerLeft = -0.6;
        targetPositionLeft = -70;
        targetPositionRight = -70;
        timeoutS = 10;
        motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight, targetPositionLeft, timeoutS, telemetry);




    }

}
