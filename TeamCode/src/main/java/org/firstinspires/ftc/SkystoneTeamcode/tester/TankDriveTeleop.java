package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="TankDriveTeleop", group = "teleop")
public class TankDriveTeleop extends LinearOpMode {

    DcMotor frontRight, frontLeft, backLeft, backRight;
    private void initialize() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight= hardwareMap.get(DcMotor.class, "backRight");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while(opModeIsActive()){
            frontRight.setPower(-gamepad1.right_stick_y);
            frontLeft.setPower(gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.right_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            }
        }

    }



