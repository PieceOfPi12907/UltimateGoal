/*package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helper.MotorHelper;

@Autonomous(name = "AutonomousTest_Encoders", group = "autonomous")
public class AutoInnerOneBlockRepoBlue extends LinearOpMode {
    //Naming the motors
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    MotorHelper motorHelper;

    public void initialize() {
        //Configuration of the motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        //setting the directions of the motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        motorHelper = new MotorHelper();
        double powerRight = 0.25;
        double powerLeft = 0.25;
        double targetPositionLeft = 12;
        double targetPositionRight = 12;
        double timeoutS = 5;
        if (opModeIsActive()) {
            motorHelper.movingWithEncoders(frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor,
                    powerRight, powerLeft,
                    targetPositionRight, targetPositionLeft,
                    timeoutS, telemetry);
        }
    }
}
*/

package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.HashMap;

//import java.util.Scanner;

@Disabled
//Adding Source Code to GitHub

public class AutoInnerTwoBlocksBlue {


    public void playProgram(HashMap<String, Object> pVariableMap) {

        Telemetry telemetry = (Telemetry) pVariableMap.get(Constants12907.TELEMETRY);
        WebcamName webcam = (WebcamName) pVariableMap.get(Constants12907.WEBCAM);
        VuforiaLocalizer.Parameters parameters = (VuforiaLocalizer.Parameters) pVariableMap.get(Constants12907.PARAMETERS);
        try {

            NavigationHelper navigationHelper = new NavigationHelper();
            SkystoneDetection skystoneDetection = new SkystoneDetection();


            Constants12907.SkystonePosition skystonePosition = (Constants12907.SkystonePosition) pVariableMap.get(Constants12907.SKY_POSITION);

            DcMotor backLeft = (DcMotor) pVariableMap.get(Constants12907.BACK_LEFT_MOTOR);
            DcMotor frontLeft = (DcMotor) pVariableMap.get(Constants12907.FRONT_LEFT_MOTOR);
            DcMotor backRight = (DcMotor) pVariableMap.get(Constants12907.BACK_RIGHT_MOTOR);
            DcMotor frontRight = (DcMotor) pVariableMap.get(Constants12907.FRONT_RIGHT_MOTOR);

            BNO055IMU imu = (BNO055IMU) pVariableMap.get(Constants12907.IMU);


            Servo pivotGrabber = (Servo) pVariableMap.get(Constants12907.PIVOT_GRABBER_SERVO);
            Servo blockClamper = (Servo) pVariableMap.get(Constants12907.BLOCK_CLAMPER_SERVO);

            Boolean isBlue = (Boolean) pVariableMap.get(Constants12907.BLUE_FLAG);
            Boolean isPlacing = (Boolean) pVariableMap.get(Constants12907.PLACING_FLAG);

            DistanceSensor quarryDistance = (DistanceSensor) pVariableMap.get(Constants12907.QUARRY_DISTANCE_SENSOR);
            //ColorSensor frontColor = (ColorSensor) pVariableMap.get(Constants12907.FRONT_COLOR_SENSOR);
            //ColorSensor backColor = (ColorSensor) pVariableMap.get(Constants12907.BACK_COLOR_SENSOR);


            telemetry.addLine("********* PROGRAM RUNNING! ********** ");
            telemetry.update();

            skystoneDetection.moveToSkystoneOne(backLeft, backRight, frontRight, frontLeft, navigationHelper, imu, telemetry, skystonePosition, quarryDistance, pivotGrabber, blockClamper, isBlue.booleanValue(), isPlacing);

            skystoneDetection.moveToSkystoneTwo(backLeft, backRight, frontRight, frontLeft, navigationHelper, imu, telemetry, skystonePosition, quarryDistance, pivotGrabber, blockClamper,  isBlue.booleanValue(), isPlacing);


        }catch (Exception bad){
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



