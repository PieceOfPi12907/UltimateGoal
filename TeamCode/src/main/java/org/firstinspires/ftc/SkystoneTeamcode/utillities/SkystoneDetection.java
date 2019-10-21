package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.helper.SensorHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SkystoneDetection {

    SensorHelper sensorHelper = new SensorHelper();
    public void moveToSkystoneOuter(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeftt){
        pNavigate.navigate(-25, Constants12907.Direction.STRAIGHT,0,-0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
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
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //Move to Intake position
        //Insert Intake Code
        pNavigate.navigate(29, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(100, Constants12907.Direction.STRAIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(5, Constants12907.Direction.LEFT,0,0.2,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(28, Constants12907.Direction.RIGHT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(31, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(-39, Constants12907.Direction.STRAIGHT,0,-0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }
}
