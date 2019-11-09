package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Test", group = "teleop")
public class IntakeTest extends LinearOpMode {
    DcMotor intakeLeft;
    DcMotor intakeRight;

    final double INTAKELEFT = 0.66;
    final double INTAKERIGHT = 0.15;
    final double DUMPERARMTOP = 0.96;
    final double DUMPERARMBOTTOM = 0.4;
    final double CLAMPDOWN = 0.9;
    final double CLAMPUP = 0.3;
    final double DUMPERROTATEIN = 0.8;
    final double DUMPERROTATEOUT = 0.17;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                intakeLeft.setPower(-0.3);
                intakeRight.setPower(0.3);
            }
            if(gamepad1.b){
                intakeLeft.setPower((0));
                intakeRight.setPower(0);
            }
        }
    }
    private void initialize(){
        intakeLeft = hardwareMap.get(DcMotor.class,"intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class,"intakeRight");
    }
}

