package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Repositioning {
    int negative;
    double leftServoDown = 1;
    double rightServoDown = 0.89;
    double leftServoUp = 0.25;
    double rightServoUp = 0.36;

    public void doRepositioning(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue, Servo repositioningRight, Servo repositioningLeft) {
        if (isBlue == false) {
            negative = -1;
        } else {
            negative = 1;
        }

        //move straight along the wall to align with foundation
        pNavigate.navigate(14*negative, Constants12907.Direction.STRAIGHT,0,0.4*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //strafe right to the foundation
        pNavigate.navigate(30, Constants12907.Direction.RIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        repositioningLeft.setPosition(leftServoDown);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        repositioningRight.setPosition(rightServoDown);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //strafe back to wall pulling foundation along
        pNavigate.navigate(32, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        repositioningLeft.setPosition(leftServoUp);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        repositioningRight.setPosition(rightServoUp);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}


