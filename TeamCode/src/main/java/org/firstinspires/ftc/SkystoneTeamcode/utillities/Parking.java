package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;


public class Parking {

   public void moveToPark(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue, HashMap pVariableMap){

       Long parkDistance = (long) pVariableMap.get(Constants12907.PARK_DISTANCE);

       pNavigate.navigate(parkDistance, Constants12907.Direction.STRAIGHT,0,0.5, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

   }
}