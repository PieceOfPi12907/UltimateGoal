package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
//@TeleOp (name = "repositioning test", group = "teleop")
public class repositioningTest extends LinearOpMode {
    Servo rightRepo;
    Servo leftRepo;
    final double LEFT_REPOSITIONING_DOWN = 0.8;
    final double LEFT_REPOSITIONING_UP = 0.1;
    final double LEFT_REPOSITIONING_MID = 0.45;
    final double RIGHT_REPOSITIONING_DOWN = 0.1;
    final double RIGHT_REPOSITIONING_MID = 0.45;
    final double RIGHT_REPOSITIONING_UP = 0.85;
    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a&&a_time.seconds()>0.25){
                a_time.reset();
                rightRepo.setPosition(RIGHT_REPOSITIONING_DOWN);
                leftRepo.setPosition(LEFT_REPOSITIONING_DOWN);
            }
            if(gamepad1.b&&b_time.seconds()>0.25){
                b_time.reset();
                rightRepo.setPosition(RIGHT_REPOSITIONING_MID);
                leftRepo.setPosition(LEFT_REPOSITIONING_MID);
            }
            if(gamepad1.x&&x_time.seconds()>0.25){
                x_time.reset();
                rightRepo.setPosition(RIGHT_REPOSITIONING_UP);
                leftRepo.setPosition(LEFT_REPOSITIONING_UP);
            }
        }
    }
    public void initialize(){
        rightRepo = hardwareMap.get(Servo.class,"rightRepositioningServo");
        leftRepo = hardwareMap.get(Servo.class, "leftRepositioningServo");
    }
}
