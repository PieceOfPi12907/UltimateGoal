package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
//@TeleOp(name = "dumper test", group= "teleop")
public class dumperSlideTest extends LinearOpMode {
    DcMotor motor;

    private void initialize() {
        motor = hardwareMap.get(DcMotor.class, "dumperMotor");


    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while ((opModeIsActive())) {
            motor.setPower(gamepad1.left_stick_y);
        }

    }

    {
    }
}
