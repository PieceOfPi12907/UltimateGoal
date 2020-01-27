package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Disabled//@TeleOp(name = "slide Test",group = "teleop")
public class SlideTest extends LinearOpMode {

    DcMotor slideMotor;
    private void initialize()
    {
        slideMotor = hardwareMap.get(DcMotor.class,"slideMotor");
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while(opModeIsActive()){

            if(gamepad2.left_stick_y>=0){
                slideMotor.setPower(-Math.pow(gamepad2.left_stick_y,2));
            }
            if(gamepad2.left_stick_y<0){
                slideMotor.setPower(Math.pow(gamepad2.left_stick_y,2));
            }
            if(gamepad2.left_stick_y==0){
                slideMotor.setPower(0);
            }

        }

    }



}