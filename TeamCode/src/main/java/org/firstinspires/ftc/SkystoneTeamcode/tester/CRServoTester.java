package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
@Disabled
//@TeleOp(name= "continuous rotation test" , group = "teleop")
public class CRServoTester extends LinearOpMode{
    private void initialize(){
        servo = hardwareMap.get(CRServo.class,"CR Servo");
        servo.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    CRServo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while ((opModeIsActive())){
            if(gamepad2.right_trigger>0){
                servo.setPower(gamepad2.right_trigger);
            }
            if(gamepad2.left_trigger>0){
                servo.setPower(-gamepad2.left_trigger);
            }
            if (gamepad2.a){
                servo.setPower(0);
            }
        }

    }
}