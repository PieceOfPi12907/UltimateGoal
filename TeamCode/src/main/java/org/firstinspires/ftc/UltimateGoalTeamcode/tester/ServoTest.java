package org.firstinspires.ftc.UltimateGoalTeamcode.tester;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

@TeleOp(name = "Servo Test 2020", group = "autonomous")
public class ServoTest extends LinearOpMode {

    Servo wobbleServo;

    private void initialize(){
        //servos
        wobbleServo = hardwareMap.get(Servo.class,"wobbleservo");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while(opModeIsActive()) {
            //wobble servo
            wobbleServo.setPosition(0.2);
            sleep(1000);
            wobbleServo.setPosition(0.8);
            telemetry.addData("WOBBLE SERVO POSITION: ", wobbleServo.getPosition());
            telemetry.update();
        }
    }
}