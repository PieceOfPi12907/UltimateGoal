package org.firstinspires.ftc.SkystoneTeamcode.tester;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
    DcMotor motor1;

    public void initializeMotor() {
        //configuring motor
        motor1 = hardwareMap.get(DcMotor.class,"Andymark 40");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //@Override
    public void runOpMode() throws InterruptedException {
        initializeMotor();
        MotorHelper motorHelper = new MotorHelper();
        waitForStart();
        if(opModeIsActive()){
            double motorPower = 0.5;
            int targetPos = 200;
            motorHelper.motorMovingWithEncoders(motor1, motorPower, targetPos, telemetry);
        }
    }
}
