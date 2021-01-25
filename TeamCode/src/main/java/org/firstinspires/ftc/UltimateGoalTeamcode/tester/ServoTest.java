package org.firstinspires.ftc.UltimateGoalTeamcode.tester;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.SkystoneTeamcode.tester.WebcamTester;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Servo Test 2020", group = "autonomous")
public class ServoTest extends LinearOpMode {

    Servo hingeServo;
    Servo clampServo;

    private void initialize(){
        //servos
        hingeServo = hardwareMap.get(Servo.class,"hinge");
        clampServo = hardwareMap.get(Servo.class,"clamp");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        if(opModeIsActive()) {

            hingeServo.setPosition(0);
            sleep(5000);
            telemetry.addData("HINGE POSITION: ", hingeServo.getPosition());
            telemetry.update();

            //CLAMP IT
            //clampServo.setPosition(0.2);
            //sleep(1000);
            telemetry.addData("CLAMP POSITION: ", clampServo.getPosition());
            telemetry.update();

            //hingeServo.setPosition(0.5);
            //sleep(1000);
            telemetry.addData("HINGE POSITION: ", hingeServo.getPosition());
            telemetry.update();

            //LET IT GO
            //clampServo.setPosition(0.5);
            //sleep(1000);
            telemetry.addData("CLAMP POSITION: ", clampServo.getPosition());
            telemetry.update();

        }
    }
}