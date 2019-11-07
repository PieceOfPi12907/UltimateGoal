package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;



public class Parking {
    int negative;

    public void parkSkystone(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue){
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }
        //ADD COLOR SENSOR CODE FOR PARKING
        pNavigate.navigate(-38*negative, Constants12907.Direction.STRAIGHT,0,-0.4*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
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
