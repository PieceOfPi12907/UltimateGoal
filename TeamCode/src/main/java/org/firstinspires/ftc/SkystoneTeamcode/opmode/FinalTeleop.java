package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;

@TeleOp(name = "FINAL TELEOP",group = "teleop")
public class FinalTeleop extends LinearOpMode {

    Servo dumperClampInsideServo;
    Servo dumperClampOutsideServo;

    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;

    DcMotor dumperMotorRight;
    DcMotor dumperMotorLeft;

    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;

    Servo sideArmServo;
    Servo sideClampServo;

    Servo dumperArmServo;

    Servo leftIntakeServo;
    Servo rightIntakeServo;

    Servo leftRepositionServo;
    Servo rightRepositionServo;

    Servo slideServo;

    Servo capServo;

    boolean isInsideClampUp = true;
    boolean isOutsideClampUp = false;
    boolean threadStarted = false;
    Constants12907.RepositioningServoPositions repositioningServoPos = Constants12907.RepositioningServoPositions.UP;
    int isSideArmDown = 1;
    boolean isSideArmClamped = false;
    boolean isIntakeSpinning = false;
    double scaleFactor = 0.9;
    double intakeSpeed = 0;
    boolean closed = false;
    boolean spinningForward = false;
    boolean slideOut = false;
    boolean clampsUp = false;

    /*final double SIDE_ARM_LOWERED = 0.75;
    final double SIDE_ARM_RAISED = 0.45;

    final double AUTO_CLAMP_OPENED = 0.4;
    final double AUTO_CLAMP_CLOSED = 0.6;*/

    final double SIDE_ARM_LOWERED = 0.8;
    final double SIDE_ARM_RAISED = 0.45;
    final double SIDE_ARM_MID = 0.7;

    final double AUTO_CLAMP_OPENED = 0.5;
    final double AUTO_CLAMP_CLOSED = 0.8;

    final double INTAKE_LEFT_OPEN = 0.68;
    final double INTAKE_RIGHT_OPEN = 0.78;
    final double INTAKE_RIGHT_CLOSE = 0.50;
    final double INTAKE_LEFT_CLOSE = 0.85;

    final double LEFT_REPOSITIONING_DOWN = 0.92;
    final double LEFT_REPOSITIONING_UP = 0.1;
    final double LEFT_REPOSITIONING_MID = 0.45;

    final double RIGHT_REPOSITIONING_DOWN = 0.01;
    final double RIGHT_REPOSITIONING_MID = 0.45;
    final double RIGHT_REPOSITIONING_UP = 0.85;

    final double DUMPER_CLAMP_INSIDE_UP = 0.825;
    final double DUMPER_CLAMP_INSIDE_DOWN = 0.38;
    final double DUMPER_CLAMP_OUTSIDE_UP = 0.1;
    final double DUMPER_CLAMP_OUTSIDE_DOWN = 0.55;

    final double SLIDE_SERVO_OUT = 0.55;
    final double SLIDE_SERVO_IN = 0.005;

    final double CAP_UP = 0.49;

    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime right_bumper_time = new ElapsedTime();
    ElapsedTime a_time_two = new ElapsedTime();
    ElapsedTime lb_time = new ElapsedTime();
    ElapsedTime dpad_time = new ElapsedTime();

    private void initialize(){
        dumperMotorRight = hardwareMap.get(DcMotor.class,"dumperMotorRight");
        dumperMotorLeft = hardwareMap.get(DcMotor.class,"dumperMotorLeft");

        dumperClampInsideServo = hardwareMap.get(Servo.class,"dumperClampInsideServo");
        dumperClampOutsideServo = hardwareMap.get(Servo.class,"dumperClampOutsideServo");
        leftIntakeMotor = hardwareMap.get(DcMotor.class,"leftIntakeMotor");
        rightIntakeMotor = hardwareMap.get(DcMotor.class,"rightIntakeMotor");
        frontRightMotor =hardwareMap.get(DcMotor.class,"frontRight");
        frontLeftMotor =hardwareMap.get(DcMotor.class, "frontLeft");
        backRightMotor =hardwareMap.get(DcMotor.class,"backRight");
        backLeftMotor =hardwareMap.get(DcMotor.class,"backLeft");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        leftRepositionServo = hardwareMap.get(Servo.class, "leftRepositioningServo");
        rightRepositionServo = hardwareMap.get(Servo.class,"rightRepositioningServo");
        sideArmServo = hardwareMap.get(Servo.class, "pivotGrabber");
        sideClampServo = hardwareMap.get(Servo.class, "blockClamper");
        slideServo = hardwareMap.get(Servo.class,"slideServo");
        capServo = hardwareMap.get(Servo.class,"capServo");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        dumperMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        dumperMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        dumperMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dumperMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntakeServo.setPosition(INTAKE_LEFT_OPEN);
        rightIntakeServo.setPosition(INTAKE_RIGHT_OPEN);
        leftRepositionServo.setPosition(LEFT_REPOSITIONING_UP);
        rightRepositionServo.setPosition(RIGHT_REPOSITIONING_UP);
        dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_UP);
        dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_DOWN);
        leftRepositionServo.setPosition(LEFT_REPOSITIONING_UP);
        repositioningServoPos = Constants12907.RepositioningServoPositions.UP;

        sideArmServo.setPosition(SIDE_ARM_RAISED);
        slideServo.setPosition(SLIDE_SERVO_IN);
        capServo.setPosition(0);

        //sideClampServo.setPosition(AUTO_CLAMP_OPENED);
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
                    newIntakeControl();
                    dumperControl();
                    testWheelControl();
                    idle();
                }
            } catch (Exception e) {

            }
        }
        private void testWheelControl(){
            if(gamepad2.y&&y_time.seconds()>0.25){
                y_time.reset();
                if(spinningForward!=true){
                    leftIntakeMotor.setPower(-0.2);
                    rightIntakeMotor.setPower(0.2);
                    spinningForward=true;
                }
                else if(spinningForward==true){
                    leftIntakeMotor.setPower(0.4);
                    rightIntakeMotor.setPower(-0.4);
                    spinningForward=false;
                }
                isIntakeSpinning=true;
            }
        }
        private void dumperControl(){
            if(gamepad2.x && x_time.seconds()>0.25){
                x_time.reset();
                if(!isInsideClampUp){
                    dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_DOWN);
                    isInsideClampUp = true;
                }else if(isInsideClampUp){
                    dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_UP);
                    isInsideClampUp = false;
                }
            }

            if(gamepad2.b && b_time.seconds()>0.25){
                b_time.reset();
                if(!isOutsideClampUp){
                    dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_DOWN);
                    isOutsideClampUp = true;
                }else if(isOutsideClampUp){
                    dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_UP);
                    isOutsideClampUp = false;
                }
            }


            if(gamepad2.left_stick_y>=0){
                dumperMotorRight.setPower(Math.pow(gamepad2.left_stick_y,2));
                dumperMotorLeft.setPower(Math.pow(gamepad2.left_stick_y,2));
            }if(gamepad2.left_stick_y<0){
                dumperMotorRight.setPower(-Math.pow(gamepad2.left_stick_y,2));
                dumperMotorLeft.setPower(-Math.pow(gamepad2.left_stick_y,2));
            }

            if(gamepad2.right_bumper && right_bumper_time.seconds()>0.25){
                right_bumper_time.reset();
                if(!slideOut){
                    slideServo.setPosition(SLIDE_SERVO_OUT);
                    slideOut = true;
                }else if(slideOut){
                    slideServo.setPosition(SLIDE_SERVO_IN);
                    slideOut = false;
                }
            }
            if(gamepad2.dpad_up&&dpad_time.seconds()>0.25){
                dpad_time.reset();
                dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_UP);
                dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_UP);

            }
            if(gamepad2.dpad_down&&dpad_time.seconds()>0.25){
                dpad_time.reset();
                dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_DOWN);
                dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_DOWN);
            }

        }

        private void newIntakeControl() {

            if(gamepad1.left_bumper&&lb_time.seconds()<0.25){
                lb_time.reset();
                leftIntakeServo.setPosition(INTAKE_LEFT_OPEN);
                rightIntakeServo.setPosition(INTAKE_RIGHT_OPEN);
            }
            if (gamepad2.a) {
                if (!isIntakeSpinning) {
                    leftIntakeMotor.setPower(-0.4);
                    rightIntakeMotor.setPower(0.4);
                    isIntakeSpinning = true;
                    dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_UP);
                    dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_DOWN);
                    telemetry.addLine("Intake is Spinning");
                    telemetry.update();
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                } else if (isIntakeSpinning) {
                    leftIntakeMotor.setPower(0);
                    rightIntakeMotor.setPower(0);
                    isIntakeSpinning = false;
                    telemetry.addLine("Intake stopped spinning");
                    telemetry.update();
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }



            }
            if(gamepad2.left_bumper&&lb_time.seconds()>0.25){
                lb_time.reset();
                if(closed!=true){
                    leftIntakeServo.setPosition(INTAKE_LEFT_CLOSE);
                    dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_UP);
                    dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_DOWN);
                    isIntakeSpinning = true;
                    closed = true;
                }
                else if(closed==true){
                    leftIntakeServo.setPosition(INTAKE_LEFT_OPEN);
                    closed = false;
                }
            }

        }


    }//end of thread class


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
            bRPower = radius * Math.cos(-3*Math.PI/4) - rotation;;
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

    /*
    GAMEPAD1.right_bumper && GAMEPAD1.left_bumper
         */




    private void repositioningControl(){
        if(gamepad1.right_bumper && right_bumper_time.seconds()>0.25){
            right_bumper_time.reset();
            if(Constants12907.RepositioningServoPositions.UP.equals(repositioningServoPos)){
                leftRepositionServo.setPosition(LEFT_REPOSITIONING_MID);
                rightRepositionServo.setPosition(RIGHT_REPOSITIONING_MID);
                repositioningServoPos = Constants12907.RepositioningServoPositions.MID;
            }else if(Constants12907.RepositioningServoPositions.MID.equals(repositioningServoPos)){
                leftRepositionServo.setPosition(LEFT_REPOSITIONING_DOWN);
                rightRepositionServo.setPosition(RIGHT_REPOSITIONING_DOWN);
                repositioningServoPos = Constants12907.RepositioningServoPositions.DOWN;

            }else if(Constants12907.RepositioningServoPositions.DOWN.equals(repositioningServoPos)){
                leftRepositionServo.setPosition(LEFT_REPOSITIONING_UP);
                rightRepositionServo.setPosition(RIGHT_REPOSITIONING_UP);
                repositioningServoPos = Constants12907.RepositioningServoPositions.UP;

            }
        }

    }

    private void sideArmControl(){
        if(gamepad1.dpad_up){
            capServo.setPosition(CAP_UP);
        }
        if(gamepad1.dpad_down && a_time.seconds() >= 0.25){
            a_time.reset();
            if(isSideArmDown==1){
                sideArmServo.setPosition(SIDE_ARM_LOWERED);
                isSideArmDown = 2;
            }
            else if(isSideArmDown==2){
                sideArmServo.setPosition(SIDE_ARM_RAISED);
                isSideArmDown = 3;
            }
            else if(isSideArmDown==3){
                sideArmServo.setPosition(SIDE_ARM_MID);
                isSideArmDown = 1;
            }
        }
        if(gamepad1.b && b_time.seconds() >= 0.25){
            b_time.reset();
            if(!isSideArmClamped){
                sideClampServo.setPosition(AUTO_CLAMP_CLOSED);
                isSideArmClamped = true;
            }
            else if(isSideArmClamped){
                sideClampServo.setPosition(AUTO_CLAMP_OPENED);
                isSideArmClamped = false;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        Thread attachments = new FinalTeleop.AttachmentsThread();
        waitForStart();
        attachments.start();
        while(opModeIsActive()){
            mecanumDrive(scaleFactor);
            if(gamepad1.x){
                scaleFactor=0.6;
            }
            if(gamepad1.y){
                scaleFactor=0.9;
            }
            repositioningControl();
            sideArmControl();
            // Code going into thread
            idle();
        }
        attachments.interrupt();
    }
}
