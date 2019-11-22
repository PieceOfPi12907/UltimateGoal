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

    public void repositioningPark(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue, Boolean isOuter){
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }

       if(isOuter != true){
           //move right (straight), out of harm's way
           pNavigate.navigate(-32*negative, Constants12907.Direction.STRAIGHT,0,-0.4*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

           //strafe "forward" (right), to stay on the inner path
           pNavigate.navigate(27, Constants12907.Direction.RIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

           //zoom right (straight), under the bridge
           pNavigate.navigate(-20, Constants12907.Direction.STRAIGHT,0,-0.4*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

       }else{
           pNavigate.navigate(-52*negative, Constants12907.Direction.STRAIGHT,0,-0.4*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
       }

    }

}
