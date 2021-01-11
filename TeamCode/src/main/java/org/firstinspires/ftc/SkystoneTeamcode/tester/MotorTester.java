package org.firstinspires.ftc.SkystoneTeamcode.tester;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.FTCappTeamcode.helper.MotorHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Autonomous(name = "MotorTester", group = "Test Programs")
public class MotorTester extends LinearOpMode {

    //name
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    public void initializeMotor() {
        //configuring motor
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backLeft = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class,"backRight");


    }

    //@Override
    public void runOpMode() throws InterruptedException {
        initializeMotor();
        waitForStart();
        if(opModeIsActive()){
            frontLeft.setPower(0.2);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            frontLeft.setPower(0);
        }
    }
}
