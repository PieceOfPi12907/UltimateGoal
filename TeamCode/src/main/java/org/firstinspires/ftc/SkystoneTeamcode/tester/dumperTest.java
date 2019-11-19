package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "dumper Test",group = "teleop")
public class dumperTest extends LinearOpMode {
    DcMotor dumperMotor;
    DcMotor slideMotor;
    private void initialize(){
        dumperMotor = hardwareMap.get(DcMotor.class,"dumperMotor");
        slideMotor = hardwareMap.get(DcMotor.class,"slideMotor");

    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.right_trigger >0.1){
                dumperMotor.setPower(Math.pow(gamepad2.right_trigger,2));
            }
            if(gamepad2.left_trigger>0.1){
                dumperMotor.setPower(-Math.pow(gamepad2.left_trigger,2));
            }
            if(gamepad2.right_trigger<0.1){
                dumperMotor.setPower(0);
            }
            if(gamepad2.left_trigger<0.1){
                dumperMotor.setPower(0);
            }
            if(gamepad2.left_stick_y>=0){
                slideMotor.setPower(-Math.pow(gamepad2.left_stick_y,2));
            }
            if(gamepad2.left_stick_y<0){
                slideMotor.setPower(Math.pow(gamepad2.left_stick_y,2));
            }

        }

    }

}
