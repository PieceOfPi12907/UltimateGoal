package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.helper.SensorHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SkystoneDetection {


    final double PIVOT_LOWERED = 0.9;
    //CHANGE VALUE FOR UPDATED ROBOT
    final double PIVOT_RAISED = 0.4;
    final double CLAMP_OPENED = 0.5;
    final double CLAMP_CLOSED = 0.8;
    int negative;



    SensorHelper sensorHelper = new SensorHelper();

    public void moveToSkystoneOuter(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeft, Servo pivotGrabber, Servo blockClamper, Boolean isBlue, Boolean isPos2) {
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }
        if(isPos2==true) {
            pNavigate.navigate(22.75*negative, Constants12907.Direction.STRAIGHT, 0, 0.5*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        }

        //Strafing to the Skystone - sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
         pNavigate.navigate(29.5, Constants12907.Direction.RIGHT, 0, 0.5, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        //Moving to the wall
      //  pNavigate.navigate(-25, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //Color Sensing Code - includes method getBlackBlock
        //boolean isLYellow = sensorHelper.isYellow(colorLeft);
        //boolean isRYellow = sensorHelper.isYellow(colorRight);
        Constants12907.SkystonePosition blackBlockPosition = sensorHelper.getBlackBlock(colorLeft,colorRight);
        //telemetry for reading the color sensor values
        pTelemetry.addData("Red Right:  ", colorRight.red());
        pTelemetry.addData("Green Right: ", colorRight.green());
        pTelemetry.addData("Red Left:  ", colorLeft.red());
        pTelemetry.addData("Green Left: ", colorLeft.green());
        pTelemetry.update();
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //OLD CODE THAT USES 'isYellow' METHOD TO DETERMINE SKYSTONE POSITION
        /*if (isRYellow && isLYellow) {
            pTelemetry.addData("position: ", 3);

        } else if (isLYellow && (isRYellow == false)) {
            pTelemetry.addData("position: ", 2);
        } else {
            pTelemetry.addData("position: ", 1);
        }
        pTelemetry.update();*/

        //RETURNS POSITION 1, 2, OR 3 FROM 'getBlackBlock' METHOD IN SensorHelper
        pTelemetry.addData("position from getBlackBlock",blackBlockPosition.toString());

        pTelemetry.addData("Distance to Stone: ", distanceRight.getDistance(DistanceUnit.CM));
        pTelemetry.update();

        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

            //robot moves to pick up skystone based on what position it is in:
            if (blackBlockPosition.equals(Constants12907.SkystonePosition.LEFT)) {
                pNavigate.navigate(5*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
                intakeSkystone(blockClamper, pivotGrabber);
                pNavigate.navigate(-5*negative, Constants12907.Direction.STRAIGHT, 0, -0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            } else if (blackBlockPosition.equals(Constants12907.SkystonePosition.CENTER) ) {
                pNavigate.navigate(2*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
                intakeSkystone(blockClamper, pivotGrabber);
                pNavigate.navigate(-2*negative, Constants12907.Direction.STRAIGHT, 0, -0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            } else {
                intakeSkystone(blockClamper, pivotGrabber);
                //does not move because the robot does not have to move anywhere for position RIGHT
            }


        //sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
     //   pNavigate.navigate(25, Constants12907.Direction.STRAIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(27, Constants12907.Direction.LEFT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(3, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }


    public void moveToSkystoneInner(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeft, Servo pivotGrabber, Servo blockClamper, Boolean isBlue, Boolean isPos2) {
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }

        //Strafing to the Skystone - sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
        pNavigate.navigate(29.5, Constants12907.Direction.RIGHT, 0, 0.5, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        if(isPos2==true) {
            pNavigate.navigate(22.75*negative, Constants12907.Direction.STRAIGHT, 0, 0.5*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        }

        //Color Sensing Code - includes method getBlackBlock
        //boolean isLYellow = sensorHelper.isYellow(colorLeft);
        //boolean isRYellow = sensorHelper.isYellow(colorRight);
        Constants12907.SkystonePosition blackBlockPosition = sensorHelper.getBlackBlock(colorLeft,colorRight);

        //telemetry for reading the color sensor values
        pTelemetry.addData("Red Right:  ", colorRight.red());
        pTelemetry.addData("Green Right: ", colorRight.green());
        pTelemetry.addData("Blue Right: ", colorRight.blue());
        pTelemetry.addData("Red Left:  ", colorLeft.red());
        pTelemetry.addData("Green Left: ", colorLeft.green());
        pTelemetry.addData("Blue Left: ", colorLeft.blue());
        pTelemetry.update();

        //OLD CODE THAT USES 'isYellow' METHOD TO DETERMINE SKYSTONE POSITION
        /*if (isRYellow && isLYellow) {
            pTelemetry.addData("position: ", 3);

        } else if (isLYellow && (isRYellow == false)) {
            pTelemetry.addData("position: ", 2);
        } else {
            pTelemetry.addData("position: ", 1);
        }
        pTelemetry.update();*/

        //RETURNS POSITION 1, 2, OR 3 FROM 'getBlackBlock' METHOD IN SensorHelper
        pTelemetry.addData("position from getBlackBlock",blackBlockPosition.toString());

        pTelemetry.addData("Distance to Stone: ", distanceRight.getDistance(DistanceUnit.CM));
        pTelemetry.update();

        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //robot moves to pick up skystone based on what position it is in:
        if(blackBlockPosition.equals(Constants12907.SkystonePosition.LEFT)){
            pNavigate.navigate(5*negative, Constants12907.Direction.STRAIGHT,0,0.25*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-5*negative, Constants12907.Direction.STRAIGHT,0,-0.25*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        } else if(blackBlockPosition.equals(Constants12907.SkystonePosition.CENTER)){
            pNavigate.navigate(2*negative, Constants12907.Direction.STRAIGHT,0,0.25*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-2*negative, Constants12907.Direction.STRAIGHT,0,-0.25*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        } else {
            intakeSkystone(blockClamper, pivotGrabber);
            //does not move because the robot does not have to move anywhere for position RIGHT
        }
    }

/*

    public void moveToSkystoneOuterRed(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeft, Servo pivotGrabber, Servo blockClamper) {
        //Strafing to the Skystone - sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
        pNavigate.navigate(29.5, Constants12907.Direction.RIGHT, 0, 0.5, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        //Moving to the wall
        //pNavigate.navigate(-25, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //Color Sensing Code - includes method getBlackBlock
        boolean isLYellow = sensorHelper.isYellow(colorLeft);
        boolean isRYellow = sensorHelper.isYellow(colorRight);
        int getBlackBlock = sensorHelper.getBlackBlock(colorLeft,colorRight);

        //telemetry for reading the color sensor values
        /*pTelemetry.addData("Red Right:  ", colorRight.red());
        pTelemetry.addData("Green Right: ", colorRight.green());
        pTelemetry.addData("Blue Right: ", colorRight.blue());
        pTelemetry.addData("Red Left:  ", colorLeft.red());
        pTelemetry.addData("Green Left: ", colorLeft.green());
        pTelemetry.addData("Blue Left: ", colorLeft.blue());
        pTelemetry.update();*/

        //OLD CODE THAT USES 'isYellow' METHOD TO DETERMINE SKYSTONE POSITION
        /*if (isRYellow && isLYellow) {
            pTelemetry.addData("position: ", 3);

        } else if (isLYellow && (isRYellow == false)) {
            pTelemetry.addData("position: ", 2);
        } else {
            pTelemetry.addData("position: ", 1);
        }
        pTelemetry.update();*/
/*
        //RETURNS POSITION 1, 2, OR 3 FROM 'getBlackBlock' METHOD IN SensorHelper
        pTelemetry.addData("position from getBlackBlock",getBlackBlock);
        pTelemetry.update();

        //robot moves to pick up skystone based on what position it is in:
        if(getBlackBlock == 3){
            pNavigate.navigate(-5, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(5, Constants12907.Direction.STRAIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        } else if(getBlackBlock == 2 ){
            pNavigate.navigate(-2, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(2, Constants12907.Direction.STRAIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        } else {
            intakeSkystone(blockClamper, pivotGrabber);
            //does not move because the robot does not have to move anywhere for position 1
        }

        //sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
        //   pNavigate.navigate(25, Constants12907.Direction.STRAIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(29, Constants12907.Direction.LEFT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(3, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }

    public void moveToSkystoneInnerRed(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeft, Servo pivotGrabber, Servo blockClamper) {
        //Strafing to the Skystone - sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
        pNavigate.navigate(29.5, Constants12907.Direction.RIGHT, 0, 0.5, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        //Moving to the wall
        //pNavigate.navigate(-25, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //Color Sensing Code - includes method getBlackBlock
        boolean isLYellow = sensorHelper.isYellow(colorLeft);
        boolean isRYellow = sensorHelper.isYellow(colorRight);
        int getBlackBlock = sensorHelper.getBlackBlock(colorLeft,colorRight);

        //telemetry for reading the color sensor values
        /*pTelemetry.addData("Red Right:  ", colorRight.red());
        pTelemetry.addData("Green Right: ", colorRight.green());
        pTelemetry.addData("Blue Right: ", colorRight.blue());
        pTelemetry.addData("Red Left:  ", colorLeft.red());
        pTelemetry.addData("Green Left: ", colorLeft.green());
        pTelemetry.addData("Blue Left: ", colorLeft.blue());
        pTelemetry.update();*/

        //OLD CODE THAT USES 'isYellow' METHOD TO DETERMINE SKYSTONE POSITION
        /*if (isRYellow && isLYellow) {
            pTelemetry.addData("position: ", 3);

        } else if (isLYellow && (isRYellow == false)) {
            pTelemetry.addData("position: ", 2);
        } else {
            pTelemetry.addData("position: ", 1);
        }
        pTelemetry.update();*/
/*
        //RETURNS POSITION 1, 2, OR 3 FROM 'getBlackBlock' METHOD IN SensorHelper
        pTelemetry.addData("position from getBlackBlock",getBlackBlock);
        pTelemetry.update();

        //robot moves to pick up skystone based on what position it is in:
        if(getBlackBlock == 3){
            pNavigate.navigate(-5, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(5, Constants12907.Direction.STRAIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        } else if(getBlackBlock == 2 ){
            pNavigate.navigate(-2, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(2, Constants12907.Direction.STRAIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        } else {
            intakeSkystone(blockClamper, pivotGrabber);
            //does not move because the robot does not have to move anywhere for position 1
        }
    }
 */


    public void intakeSkystone(Servo blockClamper, Servo pivotGrabber) {
        //Intake skystone  Code
        blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_LOWERED);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_CLOSED);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_RAISED);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}



