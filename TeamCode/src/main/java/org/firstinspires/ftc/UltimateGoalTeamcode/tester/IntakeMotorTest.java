package org.firstinspires.ftc.UltimateGoalTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "INTAKE motor test", group = "teleop")
public class IntakeMotorTest extends LinearOpMode {

    boolean intakeOn = false;
    boolean intakeBack = false;

    DcMotor intakeMotor;
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();

    public void initializeMotors(){

        intakeMotor = hardwareMap.get(DcMotor.class,"wobbleHingeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeMotors();
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                if (gamepad1.x && x_time.seconds() >= 0.25) {
                    x_time.reset();
                    if (intakeOn) {
                        intakeMotor.setPower(0);
                        intakeOn = false;
                    } else {
                        telemetry.addLine("Positive Power");
                        telemetry.update();
                        intakeMotor.setPower(0.35);
                        intakeOn = true;
                    }
                }
                if (gamepad1.y && y_time.seconds() >= 0.25) {
                    y_time.reset();
                    if (intakeBack) {
                        intakeMotor.setPower(0);
                        intakeBack = false;
                    } else {
                        telemetry.addLine("Negative Power");
                        telemetry.update();
                        intakeMotor.setPower(-0.35);
                        intakeBack = true;
                    }
                }
            }
        }
    }
}
