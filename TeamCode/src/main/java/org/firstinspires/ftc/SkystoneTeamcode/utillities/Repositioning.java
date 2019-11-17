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

    public void doRepositioning(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue, Servo repositioningRight, Servo repositioningLeft){
        if (isBlue==false) {
            negative = -1;
        } else {
            negative = 1;
        }

        pNavigate.navigate(31, Constants12907.Direction.RIGHT, 0, 0.35, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        repositioningLeft.setPosition(leftServoDown);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningRight.setPosition(rightServoDown);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        pNavigate.navigate(41, Constants12907.Direction.LEFT, 0, 0.35, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        repositioningLeft.setPosition(leftServoUp);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningRight.setPosition(rightServoUp);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}