package org.firstinspires.ftc.UltimateGoalTeamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.Constants2020;

@TeleOp(name = "FINAL ULTIMATE TELEOP",group = "teleop")
public class UltimateTeleop extends LinearOpMode {

    DcMotor shooterIntake;
    Servo shooterIntakeServo;

    DcMotor actualIntake;

    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;

    Servo wobbleHingeServo;
    Servo wobbleClampServo;
    Constants2020.HingeServoPositions hingeServoPos = Constants2020.HingeServoPositions.UP;

    boolean isClamped = true;
    double scaleFactor = 0.999;
    boolean shooterIntakeSpinning = false;
    boolean shooterIntakeOpen = false;
    boolean intakeOn = false;
    boolean intakeBack = false;

    public final double SHOOTER_INTAKE_SERVO_INIT = 0.5;
    public final double SHOOTER_INTAKE_SERVO_OPEN = 0.5;
    public final double SHOOTER_INTAKE_SERVO_CLOSE = 0.85;

    //hinge and clamp values to be tested:
    final double HINGE_SERVO_UP = 0.1; //0.2
    final double HINGE_SERVO_MID = 0.45; // 0.5
    final double HINGE_SERVO_DOWN = 0.6; // 0.8
    final double CLAMP_SERVO_OUT = 0.6;
    final double CLAMP_SERVO_IN = 0.2;
    double shooterSpeed = 0.8;

    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime rb_time = new ElapsedTime();
    ElapsedTime right_bumper_time = new ElapsedTime();
    ElapsedTime left_bumper_time = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime dpad_time = new ElapsedTime();

    private void initialize() {

        shooterIntake = hardwareMap.get(DcMotor.class, "shooterIntake");
        shooterIntakeServo = hardwareMap.get(Servo.class, "shooterIntakeServo");
        shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_INIT);
        shooterIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        actualIntake = hardwareMap.get(DcMotor.class, "intake");
        actualIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");

        wobbleClampServo = hardwareMap.get(Servo.class, "clamp");
        wobbleHingeServo = hardwareMap.get(Servo.class, "hinge");
        hingeServoPos = Constants2020.HingeServoPositions.UP;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
                    wobbleArmControl();
                    intakeControl();
                    idle();
                }
            } catch (Exception e) {

            }
        }

        private void intakeControl() {

            if (gamepad1.x && x_time.seconds() >= 0.25) {
                x_time.reset();
                if (intakeOn) {
                    actualIntake.setPower(0);
                    intakeOn = false;
                } else {
                    actualIntake.setPower(0.85);
                    intakeOn = true;
                }
            }
            if (gamepad1.y && y_time.seconds() >= 0.25) {
                y_time.reset();
                if (intakeBack) {
                    actualIntake.setPower(0);
                    intakeBack = false;
                } else {
                    actualIntake.setPower(-0.85);
                    intakeBack = true;
                }
            }
        }

        private void shooterIntakeControl() {
            if (gamepad1.dpad_up && dpad_time.seconds() > 0.1) {
                dpad_time.reset();
                shooterSpeed += 0.01;
                telemetry.addData("shooter speed is ", shooterSpeed);
                telemetry.update();
            }
            if (gamepad1.dpad_down && dpad_time.seconds() > 0.1) {
                dpad_time.reset();
                shooterSpeed -= 0.01;
                telemetry.addData("shooter speed is", shooterSpeed);
                telemetry.update();
            }
            if (gamepad1.dpad_left && dpad_time.seconds() > 0.1) {
                dpad_time.reset();
                shooterSpeed = 0.69;
            }
            if (gamepad1.dpad_right && dpad_time.seconds() > 0.1) {
                dpad_time.reset();
                shooterSpeed = 0.8;
            }
            if (gamepad1.a && a_time.seconds() >= 0.25) {
                a_time.reset();
                if (shooterIntakeSpinning) {
                    telemetry.addLine("shooter off");
                    telemetry.update();
                    shooterIntakeSpinning = false;
                    shooterIntake.setPower(0);
                } else {
                    telemetry.addLine("shooter on");
                    telemetry.update();
                    shooterIntakeSpinning = true;
                    shooterIntake.setPower(shooterSpeed);
                }
            }
            if (gamepad1.b && b_time.seconds() >= 0.25) {
                b_time.reset();
                shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_OPEN);
            }
        }

        private void wobbleArmControl() {
            if (gamepad1.right_bumper && right_bumper_time.seconds() > 0.25) {
                right_bumper_time.reset();
                if (Constants2020.HingeServoPositions.UP.equals(hingeServoPos)) {
                    wobbleHingeServo.setPosition(HINGE_SERVO_DOWN);
                    hingeServoPos = Constants2020.HingeServoPositions.MID;
                } else if (Constants2020.HingeServoPositions.MID.equals(hingeServoPos)) {
                    wobbleHingeServo.setPosition(HINGE_SERVO_MID);
                    hingeServoPos = Constants2020.HingeServoPositions.DOWN;
                } else if (Constants2020.HingeServoPositions.DOWN.equals(hingeServoPos)) {
                    wobbleHingeServo.setPosition(HINGE_SERVO_UP);
                    hingeServoPos = Constants2020.HingeServoPositions.UP;
                }
            }
            if (gamepad1.left_bumper && left_bumper_time.seconds() > 0.25) {
                left_bumper_time.reset();
                if (!isClamped) {
                    wobbleClampServo.setPosition(CLAMP_SERVO_OUT);
                    isClamped = true;
                    telemetry.addLine("SERVO IS CLAMPED");
                    telemetry.update();
                } else if (isClamped) {
                    wobbleClampServo.setPosition(CLAMP_SERVO_IN);
                    isClamped = false;
                    telemetry.addLine("SERVO IS NOT CLAMPED");
                    telemetry.update();
                }
            }
        }
    }

    //end of thread class

    //left and right bumpers: wobble arm clamp and hinge servo



    private void mecanumDrive(double scale){
        double radius=Math.hypot(gamepad2.left_stick_x,gamepad2.left_stick_y);
        double angle = (Math.atan2(-(gamepad2.left_stick_y),(gamepad2.left_stick_x)))-(Math.PI/4);
        double rotation = gamepad2.right_stick_x * 0.5;
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
            fLPower = radius * Math.cos(angle) - rotation;
            bLPower = radius * Math.sin(angle) - rotation;
            fRPower = radius * Math.sin(angle) + rotation;
            bRPower = radius * Math.cos(angle) + rotation;

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
            if(gamepad2.x){
                scaleFactor=0.6;
            }
            if(gamepad2.y){
                scaleFactor=0.99;
            }
            if(gamepad2.a){
                shooterIntake.setPower(shooterSpeed);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_OPEN);
                frontLeftMotor.setPower(0.3);
                frontRightMotor.setPower(-0.3);
                backLeftMotor.setPower(-0.3);
                backRightMotor.setPower(0.3);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_CLOSE);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_OPEN);
                shooterIntake.setPower(0);

            }
            idle();
        }
        attachments.interrupt();
    }
}
