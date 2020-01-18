package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Repositioning;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.HashMap;

//import java.util.Scanner;

//Adding source code to GitHub

@Disabled
public class AutoInnerOneBlockRepoBlue {

    Boolean isStoneRepo = true;

    public void playProgram(HashMap<String, Object> pVariableMap) {

        isStoneRepo = true;

        Telemetry telemetry = (Telemetry) pVariableMap.get(Constants12907.TELEMETRY);
        WebcamName webcam = (WebcamName) pVariableMap.get(Constants12907.WEBCAM);
        VuforiaLocalizer.Parameters parameters = (VuforiaLocalizer.Parameters) pVariableMap.get(Constants12907.PARAMETERS);
        try {

            NavigationHelper navigationHelper = new NavigationHelper();
            SkystoneDetection skystoneDetection = new SkystoneDetection();
            Repositioning repositioning = new Repositioning();

            Constants12907.SkystonePosition skystonePosition = (Constants12907.SkystonePosition) pVariableMap.get(Constants12907.SKY_POSITION);

            DcMotor backLeft = (DcMotor) pVariableMap.get(Constants12907.BACK_LEFT_MOTOR);
            DcMotor frontLeft = (DcMotor) pVariableMap.get(Constants12907.FRONT_LEFT_MOTOR);
            DcMotor backRight = (DcMotor) pVariableMap.get(Constants12907.BACK_RIGHT_MOTOR);
            DcMotor frontRight = (DcMotor) pVariableMap.get(Constants12907.FRONT_RIGHT_MOTOR);

            BNO055IMU imu = (BNO055IMU) pVariableMap.get(Constants12907.IMU);

            ElapsedTime runtime = (ElapsedTime) pVariableMap.get(Constants12907.ELAPSEDTIME);

            Servo pivotGrabber = (Servo) pVariableMap.get(Constants12907.PIVOT_GRABBER_SERVO);
            Servo blockClamper = (Servo) pVariableMap.get(Constants12907.BLOCK_CLAMPER_SERVO);
            Servo repositioningRight = (Servo) pVariableMap.get(Constants12907.RIGHT_REPOSITIONING_SERVO);
            Servo repositioningLeft = (Servo) pVariableMap.get(Constants12907.LEFT_REPOSITIONING_SERVO);


            Boolean isBlue = (Boolean) pVariableMap.get(Constants12907.BLUE_FLAG);
            Boolean isOuter = (Boolean) pVariableMap.get(Constants12907.OUTER_FLAG);


            DistanceSensor quarryDistance = (DistanceSensor) pVariableMap.get(Constants12907.QUARRY_DISTANCE_SENSOR);
            ColorSensor frontColor = (ColorSensor) pVariableMap.get(Constants12907.FRONT_COLOR_SENSOR);
            ColorSensor backColor = (ColorSensor) pVariableMap.get(Constants12907.BACK_COLOR_SENSOR);

            telemetry.addLine("********* PROGRAM RUNNING! ********** ");
            telemetry.update();

            skystoneDetection.moveToSkystoneOneWithREPO(backLeft, backRight, frontRight, frontLeft, navigationHelper, imu, telemetry, skystonePosition, quarryDistance, pivotGrabber, blockClamper,  isBlue, repositioningRight, repositioningLeft);

            repositioning.doAngleRepositioning(frontLeft, frontRight, backLeft, backRight, navigationHelper, imu, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);

        } catch (Exception bad){
            telemetry.addData("EXCEPTION:", bad.toString());
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}