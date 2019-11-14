package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "dumper Test",group = "teleop")
public class dumperTest extends LinearOpMode {
    DcMotor dumperMotor;
    private void initialize(){
        dumperMotor = hardwareMap.get(DcMotor.class,"dumperMotor");
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.right_trigger >0.1){
                dumperMotor.setPower(0.9);
            }
            if(gamepad2.left_trigger>0.1){
                dumperMotor.setPower(-0.8);
            }
            if(gamepad2.right_trigger<0.1){
                dumperMotor.setPower(0);
            }
            if(gamepad2.left_trigger<0.1){
                dumperMotor.setPower(0);
            }
        }

    }

}
