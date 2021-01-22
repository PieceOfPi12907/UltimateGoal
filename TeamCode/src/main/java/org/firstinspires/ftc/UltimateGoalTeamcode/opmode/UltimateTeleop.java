package org.firstinspires.ftc.UltimateGoalTeamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;

@TeleOp(name = "FINAL ULTIMATE TELEOP",group = "teleop")
public class UltimateTeleop extends LinearOpMode {

    DcMotor shooterIntake;
    Servo shooterIntakeServo;

    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;

    double scaleFactor = 0.999;
    boolean shooterIntakeSpinning = false;
    boolean shooterIntakeOpen = false;

    public final double SHOOTER_INTAKE_SERVO_INIT = 0.25;
    public final double SHOOTER_INTAKE_SERVO_OPEN = 0.25;
    public final double SHOOTER_INTAKE_SERVO_CLOSE = 0.6;

    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime rb_time = new ElapsedTime();

    private void initialize() {

        shooterIntake = hardwareMap.get(DcMotor.class, "shooterIntake");
        shooterIntakeServo = hardwareMap.get(Servo.class, "shooterIntakeServo");
        shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_INIT);
        shooterIntake.setDirection(DcMotorSimple.Direction.REVERSE);


        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private class AttachmentsThread extends Thread {
        boolean isIntakeSpinning = false;
        boolean isIntakeServoOpen = true;

        public void AttachmentsThread() {
            this.setName("Attachments Thread");
        }

        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                    shooterIntakeControl();
                    idle();
                }
            } catch (Exception e) {

            }
        }
        private void shooterIntakeControl(){
            if(gamepad1.a && a_time.seconds()>=0.25){
                a_time.reset();
                if(shooterIntakeSpinning){
                    shooterIntakeSpinning = false;
                    shooterIntake.setPower(0);
                }
                else {
                    shooterIntakeSpinning = true;
                    shooterIntake.setPower(1);
                }
            }
            if (gamepad1.right_bumper && rb_time.seconds()>=0.25){
                rb_time.reset();
                if(shooterIntakeOpen){
                    shooterIntakeOpen = false;
                    shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);
                }
                else{
                    shooterIntakeOpen = true;
                    shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_OPEN);
                }
            }
        }
    }
    //end of thread class


    private void mecanumDrive(double scale){
        double radius=Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y);
        double angle = (Math.atan2(-(gamepad1.left_stick_y),(gamepad1.left_stick_x)))-(Math.PI/4);
        double rotation = gamepad1.right_stick_x;
        double fLPower = 0;
        double bLPower = 0;
        double fRPower = 0;
        double bRPower = 0;

        if( (angle > 5*(Math.PI/12))&& (angle < 7*(Math.PI/12)) ){
            fLPower = radius * Math.cos(Math.PI/4) + rotation;
            bLPower = radius * Math.sin(Math.PI/4) + rotation;
            fRPower = radius * Math.sin(Math.PI/4) - rotation;
            bRPower = radius * Math.cos(Math.PI/4) - rotation;
        }
        else if( (angle < -5*(Math.PI/12))&& (angle > -7*(Math.PI/12)) ){
            fLPower = radius * Math.cos(-3*Math.PI/4) + rotation;
            bLPower = radius * Math.sin(-3*Math.PI/4) + rotation;
            fRPower = radius * Math.sin(-3*Math.PI/4) - rotation;
            bRPower = radius * Math.cos(-3*Math.PI/4) - rotation;
        }
        else {
            fLPower = radius * Math.cos(angle) + rotation;
            bLPower = radius * Math.sin(angle) + rotation;
            fRPower = radius * Math.sin(angle) - rotation;
            bRPower = radius * Math.cos(angle) - rotation;

        }
        frontLeftMotor.setPower((fLPower) * scale);
        backLeftMotor.setPower((bLPower) * scale);
        frontRightMotor.setPower((fRPower) * scale);
        backRightMotor.setPower((bRPower) * scale);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        Thread attachments = new UltimateTeleop.AttachmentsThread();
        waitForStart();
        attachments.start();
        while(opModeIsActive()){
            mecanumDrive(scaleFactor);
            if(gamepad1.x){
                scaleFactor=0.6;
            }
            if(gamepad1.y){
                scaleFactor=0.99;
            }
            idle();
        }
        attachments.interrupt();
    }
}
