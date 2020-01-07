package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;



public class Parking {

   public void moveToPark(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue){

       pNavigate.navigate(10, Constants12907.Direction.STRAIGHT,0,0.5, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

   }
}
