package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp 12907", group = "teleop")
public class SkystoneTeleop12907 extends LinearOpMode {

    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor frontLeft;
    Servo pivotGrabber;
    Servo blockClamper;

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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //pivotGrabber.setPosition(PIVOT_LOWERED);
        //blockClamper.setPosition(PIVOT_LOWERED);

    }

    public void simpleDrive(double scale){
        double radius = 1;
        //double radius=Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y);
        double rotation = gamepad1.right_stick_x;
        if(gamepad1.left_stick_y>0.5){
            frontLeft.setPower(scale*(radius+rotation));
            backLeft.setPower(scale*(radius+rotation));
            frontRight.setPower(scale*(radius-rotation));
            backRight.setPower(scale*(radius-rotation));
        }
        else if(gamepad1.left_stick_x<-0.5){
            frontLeft.setPower(-scale*(radius+rotation));
            backLeft.setPower(scale*(radius+rotation));
            frontRight.setPower(scale*(radius-rotation));
            backRight.setPower(-scale*(radius-rotation));
        }
        else if(gamepad1.left_stick_y<-0.5){
            frontLeft.setPower(-scale*(radius+rotation));
            backLeft.setPower(-scale*(radius+rotation));
            frontRight.setPower(-scale*(radius-rotation));
            backRight.setPower(-scale*(radius-rotation));
        }
        else if(gamepad1.left_stick_x>0.5){
            frontLeft.setPower(scale*(radius+rotation));
            backLeft.setPower(-scale*(radius+rotation));
            frontRight.setPower(-scale*(radius-rotation));
            backRight.setPower(scale*(radius-rotation));
        }
    }

    public void mecanumDrive(double scale){

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

    public void forwardBackwards(){
        double power = 0.7*(gamepad1.left_stick_y);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while(opModeIsActive()){


            mecanumDrive(0.5);
            /*
            frontLeft.setPower(gamepad1.left_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(gamepad1.right_stick_y);
            backRight.setPower(gamepad1.right_stick_y);
            if (gamepad1.a) {
                pivotGrabber.setPosition(PIVOT_RAISED);
            }
            if (gamepad1.b) {
                pivotGrabber.setPosition(PIVOT_LOWERED);
            }
            if (gamepad1.x) {
                blockClamper.setPosition(CLAMP_OPENED);
            }
            if(gamepad1.y){
                blockClamper.setPosition(CLAMP_CLOSED);
            }
             */

        }

        telemetry.addData("End Position: ","FL %7d : BL %7d : FR %7d : BR %7d", frontLeft.getCurrentPosition(),backLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backRight.getCurrentPosition());

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


    }
}
