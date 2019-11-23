package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Tester Teleop", group = "teleop")
public class TestingTeleop extends LinearOpMode {

    Servo latchLeft;
    Servo latchRight;
    DcMotor sweepLeft;
    DcMotor sweepRight;

    public void initialize(){

        latchLeft = hardwareMap.get(Servo.class, "latchLeft");
        latchRight = hardwareMap.get(Servo.class, "latchRight");
        sweepLeft = hardwareMap.get(DcMotor.class, "sweepLeft");
        sweepRight = hardwareMap.get(DcMotor.class, "sweepRight");

    }
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.left_bumper){
                latchLeft.setPosition(0);
                latchRight.setPosition(0);
            }
            if(gamepad1.right_bumper){
                latchLeft.setPosition(1);
                latchRight.setPosition(1);
            }
            if(gamepad1.left_trigger>0.1){
                sweepRight.setPower(0.7);
                sweepLeft.setPower(0.7);
            }
            if(gamepad1.left_trigger<=0.1){
                sweepRight.setPower(0);
                sweepLeft.setPower(0);
            }

        }
    }
}
