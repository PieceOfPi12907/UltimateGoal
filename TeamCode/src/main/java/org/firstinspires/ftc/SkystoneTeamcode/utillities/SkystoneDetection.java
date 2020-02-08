package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.helper.SensorHelper;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class SkystoneDetection {

    final double PIVOT_LOWERED = 0.8;
    final double PIVOT_MID = 0.7;
    final double PIVOT_RAISED = 0.42;

    final double CLAMP_OPENED = 0.5;
    //final double CLAMP_CLOSED = 0.8;
    final double CLAMP_CLOSED = 1;


    double leftServoDown = 0.95;
    double rightServoDown = 0.01;

    int negative;
    double correction;
    double shift;
    double imu_correct;

    NavigationHelper navigater = new NavigationHelper();
    SensorHelper sensorHelper = new SensorHelper();

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
            Thread.sleep(50);
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
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void extakeSkystone(Servo blockClamper, Servo pivotGrabber) {
        pivotGrabber.setPosition(PIVOT_LOWERED);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_RAISED);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_CLOSED);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    public void moveToSkystoneOne(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, Boolean isBlue){

        if(isBlue == true){
            negative = 1;
        } else {
            negative = -1;
        }


        if(isBlue == true){
            goToSkystoneOneBlue(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);
        } else {
            goToSkystoneOneRed(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper,isBlue);
        }

        pNavigate.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        pNavigate.navigate(23*negative, Constants12907.Direction.STRAIGHT, 0, 0.65*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        pNavigate.navigate(11*negative, Constants12907.Direction.STRAIGHT, 0, 0.40*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        extakeSkystone(blockClamper, pivotGrabber);
    }

    private void moveWithDistanceOne(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper,  Boolean isBlue){
        pivotGrabber.setPosition(0.45);

        double currentDistance = quarryDistance.getDistance(DistanceUnit.INCH);

        double targetDistance;

        if(isBlue == true){
            targetDistance = -2;
        } else {
            targetDistance = -2;
        }

        pTelemetry.addData("DISTANCE TO QUARRY: ", currentDistance);
        pTelemetry.update();

        if(currentDistance > 21){
            pTelemetry.addLine("Distance Too Great!! Stone not found");
            pTelemetry.update();
            currentDistance = 20.5;
        }

        double toGoDistance = currentDistance - targetDistance;

        pTelemetry.addData("STRAFE DISTANCE ", toGoDistance);
        pTelemetry.update();

        pivotGrabber.setPosition(PIVOT_MID);

        pNavigate.navigate(toGoDistance, Constants12907.Direction.RIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
    }

    private void goToSkystoneOneRed(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper,  Boolean isBlue){
        if (pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("position --> LEFT");

            pNavigate.navigate(8, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            moveWithDistanceOne(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(5.25, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            pNavigate.navigate(-20, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else if (pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER) ) {
            pTelemetry.addLine("position --> CENTER");

            //pNavigate.navigate(-2, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            pNavigate.navigate(0, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            moveWithDistanceOne(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(5.25, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            //pNavigate.navigate(-8, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            pNavigate.navigate(-12, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else {

            pTelemetry.addLine("position --> RIGHT");

            //pNavigate.navigate(-8, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            pNavigate.navigate(-8, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            moveWithDistanceOne(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(5.25, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            //pNavigate.navigate(-2, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            pNavigate.navigate(-4, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        }
    }

    private void goToSkystoneOneBlue(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper,  Boolean isBlue){
        if (pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("position --> LEFT");

            //Get To skystone
            pNavigate.navigate(14, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            moveWithDistanceOne(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            //Get to common point
            pNavigate.navigate(5.5, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else if (pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER) ) {
            pTelemetry.addLine("position --> CENTER");

            //Get to skystone
            pNavigate.navigate(6.5, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            moveWithDistanceOne(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            //Get to common point
            pNavigate.navigate(13, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else {

            pTelemetry.addLine("position --> RIGHT");

            //Get to skystone
            pNavigate.navigate(-1, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            moveWithDistanceOne(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            //Get to common point
            pNavigate.navigate(20.5, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        }
    }


    public void moveToSkystoneTwo(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, Boolean isBlue, ElapsedTime pRuntime){

        if(isBlue == true){
            negative = 1;
            shift = 0;

            //Move Back to Get Second Skystone (Blue)
            pNavigate.navigate(-41*negative, Constants12907.Direction.STRAIGHT, 0, -0.65*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            pNavigate.navigate(-21*negative, Constants12907.Direction.STRAIGHT, 0, -0.4*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            try {
                Thread.sleep(150);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        } else {
            negative = -1;
            //Move Back to Get Second Skystone (Red)
            pNavigate.navigate(-41*negative, Constants12907.Direction.STRAIGHT, 0, -0.65*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            pNavigate.navigate(-21*negative, Constants12907.Direction.STRAIGHT, 0, -0.4*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            try {
                Thread.sleep(150);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

       if(isBlue == true){
            goToSkystoneTwoBlue(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);
        } else {
            goToSkystoneTwoRed(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);
       }

        pNavigate.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        pNavigate.navigate(66*negative, Constants12907.Direction.STRAIGHT, 0, 0.75*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        extakeSkystone(blockClamper, pivotGrabber);

        //if(pRuntime.seconds() <= 25){
            pNavigate.navigate(-20*negative, Constants12907.Direction.STRAIGHT, 0, -0.6*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        //}
    }

    private void moveWithDistanceTwo(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper,  Boolean isBlue){

        blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        double currentDistance = quarryDistance.getDistance(DistanceUnit.INCH);

        double targetDistance = 0;

        pTelemetry.addData("**2nd DISTANCE TO QUARRY: ", currentDistance);
        pTelemetry.update();

        double toGoDistance = currentDistance - targetDistance;

        pTelemetry.addData("STRAFE DISTANCE ", toGoDistance);

        pivotGrabber.setPosition(PIVOT_MID);

        rightStrafeWithoutCorrection(toGoDistance, 0.2, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

    }

    private void goToSkystoneTwoBlue(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper,  Boolean isBlue){
        if (pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("2nd position --> LEFT");

            pNavigate.navigate(0, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            moveWithDistanceTwo(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(4, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            pNavigate.navigate(0, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else if (pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER) ) {
            pTelemetry.addLine("2nd position --> CENTER");
            pNavigate.navigate(-9.65, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            moveWithDistanceTwo(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(4, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            pNavigate.navigate(9.65, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else {
            pTelemetry.addLine("2nd position --> RIGHT");
            pNavigate.navigate(-20, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            moveWithDistanceTwo(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(4, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            pNavigate.navigate(16, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        }
    }

    private void goToSkystoneTwoRed(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper,  Boolean isBlue){
        if (pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("2nd position --> LEFT");
            pNavigate.navigate(20, Constants12907.Direction.STRAIGHT, 0, 0.6, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            moveWithDistanceTwo(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(4, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            pNavigate.navigate(-16, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else if (pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER) ) {
            pTelemetry.addLine("2nd position --> CENTER");
             pNavigate.navigate(8, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            moveWithDistanceTwo(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(4, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            pNavigate.navigate(-8, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else {
            pTelemetry.addLine("2nd position --> RIGHT");
            pNavigate.navigate(0, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            moveWithDistanceTwo(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(4, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            pNavigate.navigate(0, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        }
    }


    public void moveToSkystoneOneWithREPO(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper,  Boolean isBlue, Servo repositioningRight, Servo repositioningLeft){

        if(isBlue == true){
            negative = 1;
        } else {
            negative = -1;
        }

        if(isBlue == true){
            goToSkystoneOneBlue(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);
        } else {
            goToSkystoneOneRed(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue);
        }

        pNavigate.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        if(isBlue == true){
            //pNavigate.navigate(68*negative, Constants12907.Direction.STRAIGHT, 0, 0.60*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            pNavigate.navigate(66*negative, Constants12907.Direction.STRAIGHT, 0, 0.60*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else{
            pNavigate.navigate(69*negative, Constants12907.Direction.STRAIGHT, 0, 0.60*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        }

        rightStrafeWithoutCorrection(22, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningLeft.setPosition(leftServoDown);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningRight.setPosition(rightServoDown);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        extakeSkystone(blockClamper, pivotGrabber);

    }


    double direction;
    double firstStoneDistance = 0;
    double secondStoneDistance;
    double leftDistance = 12;
    double centerDistance = 4;
    double rightDistance = -1;

    //SET 'DIRECTION' FLAG TO -1 OR 1 BASED ON IF BLUE OR NOT AND PASS 'DIRECTION' INSTEAD OF IFBLUE
    /*
    align,
    move to grab,
    grab,
    deliver,
    place,
    align,
    grab,
    deliver,
    latch,
    place,
     */

    public void twoStonePlaceMethod(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, Boolean isBlue, ElapsedTime pRuntime, Servo repositioningRight, Servo repositioningLeft){
        if(isBlue == true){
            direction = 1;
        } else {
            direction = -1;
        }

        //ALIGN to skystone based on if it's in the left, center, or right position
        if(pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("position --> LEFT");
            pNavigate.navigate(leftDistance*direction, Constants12907.Direction.STRAIGHT, 0, 0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            secondStoneDistance = 4;
            firstStoneDistance = 0;
        }
        if(pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER)) {
            pTelemetry.addLine("position --> CENTER");
            pNavigate.navigate(centerDistance*direction, Constants12907.Direction.STRAIGHT, 0, 0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            secondStoneDistance = 12;
            firstStoneDistance = 12;
        }
        if(pSkystonePosition.equals(Constants12907.SkystonePosition.RIGHT)) {
            pTelemetry.addLine("position --> RIGHT");
            pNavigate.navigate(rightDistance*direction, Constants12907.Direction.STRAIGHT, 0, 0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            secondStoneDistance = 20;
            firstStoneDistance = 16;

        }


        //MOVE TO GRAB using fixed distance for now to get to quarry
        //(47" (wall to quarry) - 18" (robot) = 29" (move) - 4" (gap btwn quarry & robot) = 25" (final))
        pNavigate.navigate(25, Constants12907.Direction.RIGHT, 0, 0.75, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        //GRAB skystone
        intakeSkystone(blockClamper, pivotGrabber);

        //DELIVER skystone (to far side of foundation)
        pNavigate.navigate((77 + firstStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        //PLACE skystone (on foundation - so it may require a strafe right before dropping, then a strafe left to come back to the same spot)
        extakeSkystone(blockClamper, pivotGrabber);

        //GO BACK TO SECOND SET OF STONES IN QUARRY
        pNavigate.navigate((-101 - secondStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, -0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
        pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
        pTelemetry.update();
        navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

        //ALIGN back to grab second skystone ^
        /*if(pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("position --> LEFT");
            pNavigate.navigate(-secondStoneDistance*direction, Constants12907.Direction.STRAIGHT, 0, -0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);
        }
        if(pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER)) {
            pTelemetry.addLine("position --> CENTER");
            pNavigate.navigate(-secondStoneDistance*direction, Constants12907.Direction.STRAIGHT, 0, -0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);
        }
        if(pSkystonePosition.equals(Constants12907.SkystonePosition.RIGHT)) {
            pTelemetry.addLine("position --> RIGHT");
            pNavigate.navigate(-secondStoneDistance*direction, Constants12907.Direction.STRAIGHT, 0, -0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);
        }*/

        //GRAB second skystone
        intakeSkystone(blockClamper, pivotGrabber);

        //DELIVER second skystone (to close side of foundation)
        pNavigate.navigate((89.5 + secondStoneDistance)*direction, Constants12907.Direction.STRAIGHT, 0, 0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        //Strafe to foundation
        pNavigate.navigate(4, Constants12907.Direction.RIGHT, 0, 0.75*direction, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        //LATCH repositioning arms on to foundation
        repositioningLeft.setPosition(leftServoDown);
        try {
            //500 --> 250
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        repositioningRight.setPosition(rightServoDown);
        try {
            //500 --> 250
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //PLACE second skystone
        extakeSkystone(blockClamper, pivotGrabber);


    } //End of TwoStoneRepo Method


    private void leftStrafeWithoutCorrection(double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft,  BNO055IMU pImu, Telemetry telemetry) {
        ElapsedTime runtime = new ElapsedTime();

        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        int newTargetPositionFrontRight;
        int newTargetPositionFrontLeft;
        int newTargetPositionBackRight;
        int newTargetPositionBackLeft;

        double startingAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionBackRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontLeft = pFrontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontRight = pFrontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(newTargetPositionBackLeft);
        pBackRight.setTargetPosition(-(newTargetPositionBackRight));
        pFrontRight.setTargetPosition(newTargetPositionFrontRight);
        pFrontLeft.setTargetPosition(-(newTargetPositionBackLeft));
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition(), pFrontLeft.getCurrentPosition(), pFrontRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pFrontRight.setPower((pSpeed));
        pBackRight.setPower(-(pSpeed));
        pFrontLeft.setPower(-(pSpeed));
        pBackLeft.setPower((pSpeed));

        telemetry.addData("Path1", "Target Position %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);

        telemetry.update();

        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {
            // Display it for the driver.
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition(),
                pFrontLeft.getCurrentPosition(),
                pFrontRight.getCurrentPosition());
        telemetry.update();
    }

    private void rightStrafeWithoutCorrection(double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft,  BNO055IMU pImu, Telemetry telemetry) {
        ElapsedTime runtime = new ElapsedTime();

        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        int newTargetPositionFrontRight;
        int newTargetPositionFrontLeft;
        int newTargetPositionBackRight;
        int newTargetPositionBackLeft;

        double startingAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionBackRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontLeft = pFrontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontRight = pFrontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(-(newTargetPositionBackLeft));
        pBackRight.setTargetPosition(newTargetPositionBackRight);
        pFrontRight.setTargetPosition(-(newTargetPositionFrontRight));
        pFrontLeft.setTargetPosition(newTargetPositionBackLeft);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition(), pFrontLeft.getCurrentPosition(), pFrontRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pFrontRight.setPower(-(pSpeed));
        pBackRight.setPower((pSpeed));
        pFrontLeft.setPower((pSpeed));
        pBackLeft.setPower(-(pSpeed));

        telemetry.addData("Path1", "Target Position %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);
        telemetry.update();

        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {

            // Display it for the driver.
            /*telemetry.addData("Path1", "Running to %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    pBackLeft.getCurrentPosition(),
                    pBackRight.getCurrentPosition(),
                    pFrontLeft.getCurrentPosition(),
                    pFrontRight.getCurrentPosition());
            telemetry.update();*/
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition(),
                pFrontLeft.getCurrentPosition(),
                pFrontRight.getCurrentPosition());
        telemetry.update();
    }
}