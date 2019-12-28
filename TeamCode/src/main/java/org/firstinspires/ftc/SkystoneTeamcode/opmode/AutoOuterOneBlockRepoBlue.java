package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDelivery;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Repositioning;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Parking;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//import java.util.Scanner;

@Disabled
//Adding Source Code to GitHub



public class AutoOuterOneBlockRepoBlue {

    Boolean isStoneRepo = true;


    public void playProgram(HashMap<String, Object> pVariableMap) {

        isStoneRepo = true;

        Telemetry telemetry = (Telemetry) pVariableMap.get(Constants12907.TELEMETRY);
        WebcamName webcam = (WebcamName) pVariableMap.get(Constants12907.WEBCAM);
        VuforiaLocalizer.Parameters parameters = (VuforiaLocalizer.Parameters) pVariableMap.get(Constants12907.PARAMETERS);

        try {

            NavigationHelper navigationHelper = new NavigationHelper();
            SkystoneDetection skystoneDetection = new SkystoneDetection();
            SkystoneDelivery skystoneDelivery = new SkystoneDelivery();
            Repositioning repositioning = new Repositioning();

            Constants12907.SkystonePosition skystonePosition = (Constants12907.SkystonePosition) pVariableMap.get(Constants12907.SKY_POSITION);

            DcMotor backLeft = (DcMotor) pVariableMap.get(Constants12907.BACK_LEFT_MOTOR);
            DcMotor frontLeft = (DcMotor) pVariableMap.get(Constants12907.FRONT_LEFT_MOTOR);
            DcMotor backRight = (DcMotor) pVariableMap.get(Constants12907.BACK_RIGHT_MOTOR);
            DcMotor frontRight = (DcMotor) pVariableMap.get(Constants12907.FRONT_RIGHT_MOTOR);

            BNO055IMU imu = (BNO055IMU) pVariableMap.get(Constants12907.IMU);

            Servo pivotGrabber = (Servo) pVariableMap.get(Constants12907.PIVOT_GRABBER_SERVO);
            Servo blockClamper = (Servo) pVariableMap.get(Constants12907.BLOCK_CLAMPER_SERVO);
            Servo repositioningRight = (Servo) pVariableMap.get(Constants12907.RIGHT_REPOSITIONING_SERVO);
            Servo repositioningLeft = (Servo) pVariableMap.get(Constants12907.LEFT_REPOSITIONING_SERVO);


            Boolean isBlue = (Boolean) pVariableMap.get(Constants12907.BLUE_FLAG);
            Boolean isOuter = (Boolean) pVariableMap.get(Constants12907.OUTER_FLAG);

//uncomment below
            DistanceSensor quarryDistance = (DistanceSensor) pVariableMap.get(Constants12907.QUARRY_DISTANCE_SENSOR);
            ColorSensor frontColor = (ColorSensor) pVariableMap.get(Constants12907.FRONT_COLOR_SENSOR);
            ColorSensor backColor = (ColorSensor) pVariableMap.get(Constants12907.BACK_COLOR_SENSOR);


//uncomment below
            telemetry.addLine("********* PROGRAM RUNNING! ********** ");
            telemetry.update();

            //navigationHelper.navigate(11, Constants12907.Direction.RIGHT,0,0.5,backLeft, backRight, frontRight, frontLeft, imu, telemetry);

            //Constants12907.SkystonePosition skystonePosition = skystoneDetection.detectSkystoneWithWebcam(telemetry,webcam, parameters);

            skystoneDetection.moveToSkystoneOneWithREPO(backLeft, backRight, frontRight, frontLeft, navigationHelper, imu, telemetry, skystonePosition, quarryDistance, pivotGrabber, blockClamper, skystoneDelivery, isBlue, repositioningRight,repositioningLeft);

            repositioning.doAngleRepositioning(frontLeft, frontRight, backLeft, backRight, navigationHelper, imu, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft);

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
