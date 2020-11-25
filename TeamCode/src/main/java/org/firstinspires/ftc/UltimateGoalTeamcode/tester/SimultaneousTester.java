package org.firstinspires.ftc.UltimateGoalTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "simultaneous", group = "auto")
public class SimultaneousTester extends LinearOpMode {

    DcMotor motor;
    Servo attachment;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if(opModeIsActive()){
            //motor.setPower(0.5);
            while(motor.getCurrentPosition()<5000 && opModeIsActive()){
                //motor.setPower(0.5);
                telemetry.addData("current pos: ", motor.getCurrentPosition());
                telemetry.update();
                //motor.setPower(0.5);
                if(attachment.getPosition()>0.52|| attachment.getPosition()<0.48){
                    attachment.setPosition(0.5);
                    sleep(100);
                }

            }
        }
    }

    public void initialize(){
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Initialization: ", "is done");
        telemetry.update();
        attachment = hardwareMap.get(Servo.class, "servo");
        attachment.setPosition(0.2);
        sleep(100);
    }
}
