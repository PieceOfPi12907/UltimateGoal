package org.firstinspires.ftc.SkystoneTeamcode.Mechanum_Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mechanum Test Run", group = "teleop")
public class Mechanum_Test extends LinearOpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    Mechanum_Helper mh = new Mechanum_Helper();

    public void initialize(){
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
     }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                mh.moveForwardForTime(frontRight, frontLeft, backRight, backLeft, 0.5, 1);
            }
            if (gamepad1.b) {
                mh.strafeRightForTime(frontRight, frontLeft, backRight, backLeft, 0.5, 1);
            }
            if (gamepad1.x) {
                mh.strafeLeftForTime(frontRight, frontLeft, backRight, backLeft, 0.5, 1);
            }
            if (gamepad1.y) {
                mh.moveForwardForTime(frontRight, frontLeft, backRight, backLeft, -0.5, 1);
            }
        }
    }
}
