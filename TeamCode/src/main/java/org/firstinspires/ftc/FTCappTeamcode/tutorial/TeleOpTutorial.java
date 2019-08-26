package org.firstinspires.ftc.teamcode.tutorial;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



@Disabled
@TeleOp(name = "Teleop Tutorial", group = "Tutorials")
public class TeleOpTutorial  extends LinearOpMode {

    private DcMotor motorPractice;

    private Servo servoPractice;

    private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTENDED_POSITION = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        motorPractice = hardwareMap.dcMotor.get("motorPractice");
        motorPractice.setDirection(DcMotor.Direction.FORWARD);

        servoPractice = hardwareMap.servo.get("servoPractice");
        servoPractice.setPosition(ARM_RETRACTED_POSITION);

        waitForStart();

        while(opModeIsActive()){
            motorPractice.setPower(-gamepad1.right_stick_y);

            if(gamepad1.a){
                servoPractice.setPosition(ARM_EXTENDED_POSITION);
            }
            if(gamepad1.b){
                servoPractice.setPosition(ARM_RETRACTED_POSITION);
            }


            idle();
        }
    }
}