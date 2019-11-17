package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;



public class Parking {
    int negative;
    final double REPOSITIONING_DOWN = 0.75;
    final double REPOSITIONING_UP = 0.15;

    public void parkSkystone(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue){
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }
        //ADD COLOR SENSOR CODE FOR PARKING
        pNavigate.navigate(-42*negative, Constants12907.Direction.STRAIGHT,0,-0.4*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }





    public void doRepositioning(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue, Servo repositioning){
        if(isBlue == false){
            pNavigate.navigate(14, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        } else {
            pNavigate.navigate(14, Constants12907.Direction.RIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        }
        pNavigate.navigate(-30, Constants12907.Direction.STRAIGHT,0,-0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        repositioning.setPosition(REPOSITIONING_DOWN);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        pNavigate.navigate(32, Constants12907.Direction.STRAIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        repositioning.setPosition(REPOSITIONING_UP);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(isBlue == false){
            pNavigate.navigate(52, Constants12907.Direction.RIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        } else {
            pNavigate.navigate(52, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        }
    }

/*
    public void parkSkystoneInner(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue, Boolean isPos2){
        //ADD COLOR SENSOR CODE FOR PARKING
        pNavigate.navigate(-38, Constants12907.Direction.STRAIGHT,0,-0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }

    public void parkSkystoneOuterRed(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry){
        //ADD COLOR SENSOR CODE FOR PARKING
        pNavigate.navigate(38, Constants12907.Direction.STRAIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }

    public void parkSkystoneInnerRed(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry){
        //ADD COLOR SENSOR CODE FOR PARKING
        pNavigate.navigate(38, Constants12907.Direction.STRAIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }
*/


}
