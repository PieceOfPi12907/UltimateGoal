package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SkystoneDelivery {

    final double PIVOT_LOWERED = 0.75;
    final double PIVOT_RAISED = 0.45;

    final double CLAMP_OPENED = 0.4;
    final double CLAMP_CLOSED = 0.6;

    int negative;
    double correction;
    double shift;

    public void placeSkystoneOuter(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Servo blockClamper, Servo pivotGrabber, Boolean isBlue){
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }
        pNavigate.navigate(78*negative, Constants12907.Direction.STRAIGHT,0,0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //below line is commented out so the robot doesn't move to the wall after moving to the building zone
        //pNavigate.navigate(5, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(31, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        extakeSkystone(blockClamper, pivotGrabber);

        pNavigate.navigate(31, Constants12907.Direction.LEFT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(3, Constants12907.Direction.RIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }


    public void placeSkystoneInner(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Servo blockClamper, Servo pivotGrabber, Boolean isBlue){
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }

        pNavigate.navigate(78*negative, Constants12907.Direction.STRAIGHT,0,0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //below line is commented out so the robot doesn't move to the wall after moving to the building zone
        //pNavigate.navigate(5, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(6, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //above line: fix distance

        extakeSkystone(blockClamper, pivotGrabber);

        //pNavigate.navigate(31, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(6, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }

    public void placeSkystoneInnerTwo(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Servo blockClamper, Servo pivotGrabber, Boolean isBlue) {
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;

        }

        //below pTgt distance was changed from 78 to 68, as the robot doesn't have to move all the way to the foundation
        pNavigate.navigate(68*negative, Constants12907.Direction.STRAIGHT,0,0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //below pTgt distance was changed from 6 to 3, since the robot doesn't have to strafe all the way to the foundation
        pNavigate.navigate(3, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //dump le Skystone ON THE MAT (not foundation)
        extakeSkystone(blockClamper, pivotGrabber);

        //back up (don't hit the skybridge)
        pNavigate.navigate(3, Constants12907.Direction.LEFT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //return to previous spot, where the robot picked up the original skystone
        pNavigate.navigate(-68*negative, Constants12907.Direction.STRAIGHT,0,-0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //move distance to the correct skystone (2), speed is slowed down.
        pNavigate.navigate(-25*negative, Constants12907.Direction.STRAIGHT,0,-0.5*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //grab skystone (#2)
        intakeSkystone(blockClamper, pivotGrabber);

        //move back to le foundation
        pNavigate.navigate(103*negative, Constants12907.Direction.STRAIGHT,0,0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //throw (place) skystone onto the foundation
        extakeSkystone(blockClamper, pivotGrabber);


        //THE BELOW LINE IS COMMENTED OUT 'cause we no longer need to back up (go to repositioning next)
        pNavigate.navigate(6, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

    }




    /*public void placeDoubleSkystone(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Servo blockClamper, Servo pivotGrabber, Boolean isBlue){
        Constants12907.SkystonePosition

        if (isBlue==false) {
            negative = -1;
            correction = 2;
            shift = 5;
        } else {
            negative = 1;
            correction = 0;
            shift = 0;
        }

        pNavigate.navigate(65*negative, Constants12907.Direction.STRAIGHT,0,0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        extakeSkystone(blockClamper, pivotGrabber);

        pNavigate.navigate(-(65*negative), Constants12907.Direction.STRAIGHT,0,0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        pNavigate.navigate(-25, Constants12907.Direction.STRAIGHT,0,0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        if (blackBlockPosition.equals(Constants12907.SkystonePosition.LEFT)) {

            pNavigate.navigate((15-correction)*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-(13-correction)*negative, Constants12907.Direction.STRAIGHT, 0, -0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else if (blackBlockPosition.equals(Constants12907.SkystonePosition.CENTER) ) {

            pNavigate.navigate((8-correction)*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-(6-correction)*negative, Constants12907.Direction.STRAIGHT, 0, -0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else {

            pNavigate.navigate((shift-correction)*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate((2-(correction+shift))*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        }

    }*/





/*
    public void placeSkystoneOuterRed(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Servo blockClamper, Servo pivotGrabber){
        pNavigate.navigate(-90, Constants12907.Direction.STRAIGHT,0,-0.7,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //below line is commented out so the robot doesn't move to the wall after moving to the building zone
        //pNavigate.navigate(5, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(28, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        extakeSkystone(blockClamper, pivotGrabber);

        pNavigate.navigate(31, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(3, Constants12907.Direction.RIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }

    public void placeSkystoneInnerRed(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Servo blockClamper, Servo pivotGrabber){
        pNavigate.navigate(-90, Constants12907.Direction.STRAIGHT,0,-0.7,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //below line is commented out so the robot doesn't move to the wall after moving to the building zone
        //pNavigate.navigate(5, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(3, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        extakeSkystone(blockClamper, pivotGrabber);

      //  pNavigate.navigate(31, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(-3, Constants12907.Direction.RIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }
*/

    public void extakeSkystone(Servo blockClamper, Servo pivotGrabber) {
        pivotGrabber.setPosition(PIVOT_LOWERED);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_RAISED);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_CLOSED);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void intakeSkystone(Servo blockClamper, Servo pivotGrabber) {
        //Intake skystone  Code
        blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_LOWERED);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_CLOSED);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_RAISED);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}

