package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "test", group = "teleop")
public class JustTeleop extends LinearOpMode {
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor frontLeft;
    Servo pivotGrabber;
    Servo blockClamper;

    double scaleFactor = 0.75;

    final double PIVOT_LOWERED = 0.15;
    final double PIVOT_RAISED = 0.8;
    final double CLAMP_OPENED = 0.5;
    final double CLAMP_CLOSED = 0.8;

    public void initialize(){

        frontRight=hardwareMap.get(DcMotor.class,"frontRight");
        frontLeft=hardwareMap.get(DcMotor.class, "frontLeft");
        backRight=hardwareMap.get(DcMotor.class,"backRight");
        backLeft=hardwareMap.get(DcMotor.class,"backLeft");
        pivotGrabber=hardwareMap.get(Servo.class, "pivotGrabber");
        blockClamper=hardwareMap.get(Servo.class, "blockClamper");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        pivotGrabber.setPosition(PIVOT_RAISED);
        blockClamper.setPosition(CLAMP_CLOSED);

    }
    public void justTest(double scale){
        double radius=Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y);
        double angle = (Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x))-(Math.PI/4);
        double rotation = gamepad1.right_stick_x;

        double fLPower = radius * Math.cos(angle) + rotation;
        double bLPower = radius * Math.sin(angle) + rotation;
        double fRPower = radius * Math.sin(angle) - rotation;
        double bRPower = radius * Math.cos(angle) - rotation;

        frontLeft.setPower((fLPower)*scale);
        backLeft.setPower((bLPower)*scale);
        frontRight.setPower((fRPower)*scale);
        backRight.setPower((bRPower)*scale);
    }
    public void lessCalc(double scale){
        double rotation = gamepad1.right_stick_x;
        if(gamepad1.left_stick_x<0.2&&gamepad1.left_stick_x>-0.2){
            frontLeft.setPower(-gamepad1.left_stick_y*scale+ rotation);
            backLeft.setPower(-gamepad1.left_stick_y*scale+ rotation);
            frontRight.setPower(-gamepad1.left_stick_y*scale- rotation);
            backRight.setPower(-gamepad1.left_stick_y*scale- rotation);
        }
        if(gamepad1.left_stick_y<0.2&&gamepad1.left_stick_y>-0.2){
                frontLeft.setPower(gamepad1.left_stick_x*scale+ rotation);
                backLeft.setPower(-gamepad1.left_stick_x*scale+ rotation);
                frontRight.setPower(-gamepad1.left_stick_x*scale- rotation);
                backRight.setPower(gamepad1.left_stick_x*scale- rotation);

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive()){
            while(gamepad1.left_stick_y!=0||gamepad1.left_stick_x!=0||gamepad1.right_stick_x!=0){
                lessCalc(scaleFactor);
            }
            if(gamepad1.a){
                scaleFactor=0.5;
            }
            if(gamepad1.b){
                scaleFactor=0.75;
            }
            if (gamepad2.a) {
                pivotGrabber.setPosition(PIVOT_RAISED);
            }
            if (gamepad2.b) {
                pivotGrabber.setPosition(PIVOT_LOWERED);
            }
            if (gamepad2.x) {
                blockClamper.setPosition(CLAMP_OPENED);
            }
            if(gamepad2.y){
                blockClamper.setPosition(CLAMP_CLOSED);
            }

            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

        }
    }
}
