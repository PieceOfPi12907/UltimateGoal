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

public class SkystoneDetection {

    final double PIVOT_LOWERED = 0.15;
    final double PIVOT_RAISED = 0.8;
    final double CLAMP_OPENED = 0.5;
    final double CLAMP_CLOSED = 0.8;


    SensorHelper sensorHelper = new SensorHelper();

    public void moveToSkystoneOuterBlue(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeft, Servo pivotGrabber, Servo blockClamper) {
        //Moving to the wall
        //pNavigate.navigate(-25, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //Strafing to the Skystone - sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
       // pNavigate.navigate(29.5, Constants12907.Direction.RIGHT, 0, 0.5, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        //Color Sensing Code

        boolean is2Yellow = sensorHelper.isYellow(colorLeft);
        boolean is1Yellow = sensorHelper.isYellow(colorRight);
        pTelemetry.addData("Red Left:  ", colorLeft.red());
        pTelemetry.addData("Green Left: ", colorLeft.green());
        pTelemetry.addData("Blue Left: ", colorLeft.blue());


        pTelemetry.addData("Red Right:  ", colorRight.red());
        pTelemetry.addData("Green Right: ", colorRight.green());
        pTelemetry.addData("Blue Right: ", colorRight.blue());

        pTelemetry.update();

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if (is1Yellow && is2Yellow) {
            pTelemetry.addData("position: ", 3);
        } else if (is1Yellow && is2Yellow == false) {
            pTelemetry.addData("position: ", 2);
        } else {
            pTelemetry.addData("position: ", 1);
        }
        pTelemetry.update();
        //Move to Intake position (not necessary, initial position is fine)
        //Intake Code
       /* blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_LOWERED);
        try {
            Thread.sleep(750);
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
        //sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
        pNavigate.navigate(29, Constants12907.Direction.LEFT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(80, Constants12907.Direction.STRAIGHT,0,0.7,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(5, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(28, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //"Extake" Code
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

        pNavigate.navigate(31, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(-30, Constants12907.Direction.STRAIGHT,0,-0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }




   public void moveToSkystoneOuterRed (DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeft, Servo pivotGrabber, Servo blockClamper ){

        //Moving to the wall
        //pNavigate.navigate(25, Constants12907.Direction.STRAIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //Strafing to the Skystone
        pNavigate.navigate(29, Constants12907.Direction.RIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //Insert Color Sensing Code
        boolean is2Yellow = sensorHelper.isYellow(colorLeft);
        boolean is1Yellow = sensorHelper.isYellow(colorRight);
        if(is1Yellow&&is2Yellow){
            pTelemetry.addData("position: ",3);
        }else if(is1Yellow&&!(is2Yellow)){
            pTelemetry.addData("position: ",2);
        }else{
            pTelemetry.addData("position: ",1);
        }
        pTelemetry.update();
        //Move to Intake position (not necessary, initial position is fine)
        //Intake Code
        blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_LOWERED);
        try {
            Thread.sleep(750);
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

        pNavigate.navigate(29, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        //pNavigate.navigate(-100, Constants12907.Direction.STRAIGHT,0,-0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
       pNavigate.navigate(-80, Constants12907.Direction.STRAIGHT,0,-0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
       pNavigate.navigate(5, Constants12907.Direction.LEFT,0,0.2,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(28, Constants12907.Direction.RIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //"Extake" Code
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


       pNavigate.navigate(31, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(32, Constants12907.Direction.STRAIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }

        */


    }
}


