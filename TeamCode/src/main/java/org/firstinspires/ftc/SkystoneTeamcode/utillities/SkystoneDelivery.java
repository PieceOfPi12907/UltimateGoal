package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SkystoneDelivery {

    final double PIVOT_LOWERED = 0.9;
    //CHANGE VALUE FOR UPDATED ROBOT
    final double PIVOT_RAISED = 0.4;
    final double CLAMP_OPENED = 0.5;
    final double CLAMP_CLOSED = 0.8;
    int negative;

    public void placeSkystoneOuter(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Servo blockClamper, Servo pivotGrabber, Boolean isBlue){
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }
        pNavigate.navigate(80*negative, Constants12907.Direction.STRAIGHT,0,0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //below line is commented out so the robot doesn't move to the wall after moving to the building zone
        //pNavigate.navigate(5, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(29, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        extakeSkystone(blockClamper, pivotGrabber);

        pNavigate.navigate(31, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(3, Constants12907.Direction.RIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }


    public void placeSkystoneInner(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Servo blockClamper, Servo pivotGrabber, Boolean isBlue){
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }
        pNavigate.navigate(90*negative, Constants12907.Direction.STRAIGHT,0,0.7*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //below line is commented out so the robot doesn't move to the wall after moving to the building zone
        //pNavigate.navigate(5, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(3, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //above line: fix distance

        extakeSkystone(blockClamper, pivotGrabber);

        //pNavigate.navigate(31, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(-3, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }
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
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_RAISED);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_CLOSED);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}

