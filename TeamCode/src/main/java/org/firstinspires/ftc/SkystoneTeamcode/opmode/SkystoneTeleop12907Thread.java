package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;

@TeleOp(name= "LEAGUE 2 - FINAL (thread) TELEOP 12907", group = "teleop")
public class SkystoneTeleop12907Thread extends LinearOpMode {
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;
    Servo sideArmServo;
    Servo sideClampServo;
    Servo dumperArmServo;
    Servo dumperClampInsideServo;
    Servo dumperClampOutsideServo;
    Servo leftIntakeServo;
    Servo rightIntakeServo;
    Servo leftRepositionServo;
    Servo rightRepositionServo;
    Servo parkingArmServo;


    boolean threadStarted = false;
    Constants12907.RepositioningServoPositions repositioningServoPos = Constants12907.RepositioningServoPositions.UP;
    Constants12907.DumperArmDropPositions dumperArmDropPositions = Constants12907.DumperArmDropPositions.DOWN;
    boolean isInsideClampUp = true;
    boolean isOutsideClampUp = false;
    boolean isArmIn = true;
    boolean isSideArmDown = false;
    boolean isSideArmClamped = false;
    boolean isIntakeSpinning = false;
    double scaleFactor = 0.7;
    double intakeSpeed = 0;

    final double SIDE_ARM_LOWERED = 0.9;
    final double SIDE_ARM_RAISED = 0.4;
    final double AUTO_CLAMP_OPENED = 0.5;
    final double AUTO_CLAMP_CLOSED = 0.8;
    final double INTAKE_LEFT_OPEN = 0.71;
    final double INTAKE_RIGHT_OPEN = 0.8;
    // final double INTAKE_LEFT_OPEN = 0;
    final double INTAKE_RIGHT_CLOSE = 0.50;
    final double DUMPER_ARM_DOWN = 1;
    final double DUMPER_ARM_DROP_1 = 0.1;
    final double DUMPER_ARM_DROP_2 = 0.25;
    final double DUMPER_ARM_DROP_3 = 0.5;
    final double LEFT_REPOSITIONING_DOWN = 0;
    final double LEFT_REPOSITIONING_UP = 0.8;
    final double LEFT_REPOSITIONING_MID = 0.3;
    final double RIGHT_REPOSITIONING_DOWN = 0.9;
    final double RIGHT_REPOSITIONING_MID = 0.75;
    final double RIGHT_REPOSITIONING_UP = 0.3;
    final double DUMPER_CLAMP_INSIDE_UP = 0.5;
    //final double DUMPER_CLAMP_INSIDE_DOWN = 0.15
    final double DUMPER_CLAMP_INSIDE_DOWN = 0.05;
    final double DUMPER_CLAMP_OUTSIDE_UP = 0.3;
    final double DUMPER_CLAMP_OUTSIDE_DOWN = 0.9;
    final double PARKING_ARM_OUT = 0.8;

    boolean goingReverse = false;
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime right_bumper_time = new ElapsedTime();
    ElapsedTime a_time_two = new ElapsedTime();




    private void initialize(){
        frontRightMotor =hardwareMap.get(DcMotor.class,"frontRight");
        frontLeftMotor =hardwareMap.get(DcMotor.class, "frontLeft");
        backRightMotor =hardwareMap.get(DcMotor.class,"backRight");
        backLeftMotor =hardwareMap.get(DcMotor.class,"backLeft");
        //pivotGrabber=hardwareMap.get(Servo.class, "pivotGrabber");
        //blockClamper=hardwareMap.get(Servo.class, "blockClamper");

        leftIntakeMotor = hardwareMap.get(DcMotor.class,"intakeLeft");
        rightIntakeMotor = hardwareMap.get(DcMotor.class,"intakeRight");

        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        dumperArmServo = hardwareMap.get(Servo.class,"dumperArm");
        dumperClampInsideServo = hardwareMap.get(Servo.class,"dumperClampInsideServo");
        dumperClampOutsideServo = hardwareMap.get(Servo.class,"dumperClampOutsideServo");
        leftRepositionServo = hardwareMap.get(Servo.class, "leftRepositioningServo");
        rightRepositionServo = hardwareMap.get(Servo.class,"rightRepositioningServo");
        sideArmServo = hardwareMap.get(Servo.class, "pivotGrabber");
        sideClampServo = hardwareMap.get(Servo.class, "blockClamper");
        parkingArmServo = hardwareMap.get(Servo.class,"parkingArmServo");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntakeServo.setPosition(INTAKE_LEFT_OPEN);
        rightIntakeServo.setPosition(INTAKE_RIGHT_OPEN);
        leftRepositionServo.setPosition(LEFT_REPOSITIONING_UP);
        rightRepositionServo.setPosition(RIGHT_REPOSITIONING_UP);
        dumperArmServo.setPosition(DUMPER_ARM_DOWN);
        dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_UP);
        dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_DOWN);
        leftRepositionServo.setPosition(LEFT_REPOSITIONING_UP);
        dumperArmDropPositions = Constants12907.DumperArmDropPositions.DOWN;
        repositioningServoPos = Constants12907.RepositioningServoPositions.UP;
        sideArmServo.setPosition(SIDE_ARM_RAISED);
        sideClampServo.setPosition(AUTO_CLAMP_CLOSED);
        //parkingArmServo.setPosition(PARKING_ARM_OUT);

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
                    idle();
                }
            } catch (Exception e) {

            }
        }

        /*
           GAMEPAD2.a
         */
        private void intakeControl() {
            if (gamepad2.a) {
                if (isIntakeServoOpen && !isIntakeSpinning) {
                    leftIntakeMotor.setPower(-0.4);
                    rightIntakeMotor.setPower(0.4);
                    isIntakeSpinning = true;
                    telemetry.addLine("Intake is Spinning");
                    telemetry.update();
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                } else if (isIntakeServoOpen && isIntakeSpinning) {
                    leftIntakeMotor.setPower(0);
                    rightIntakeMotor.setPower(0);
                    rightIntakeServo.setPosition(INTAKE_RIGHT_CLOSE);

                    isIntakeSpinning = false;
                    isIntakeServoOpen = false;
                    telemetry.addLine("Intake stopped spinning and intake servo is closed");
                    telemetry.update();
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                } else if (!isIntakeServoOpen && !isIntakeSpinning) {
                    leftIntakeServo.setPosition(INTAKE_LEFT_OPEN);
                    rightIntakeServo.setPosition(INTAKE_RIGHT_OPEN);
                    isIntakeServoOpen = true;
                    isIntakeSpinning = false;
                    telemetry.addLine("Intake servo is open and Intake is not spinning");
                    telemetry.update();
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                } else {
                    telemetry.addLine("Intake Servo and Motor error");
                    telemetry.update();
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                telemetry.update();

            }

        }

        private void newIntakeControl() {
            if (gamepad2.a) {
                if (!isIntakeSpinning) {
                    leftIntakeMotor.setPower(-0.4);
                    rightIntakeMotor.setPower(0.4);
                    isIntakeSpinning = true;
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

        }

        private void dumperControl() {

            if(gamepad2.x && x_time.seconds()>0.25){
                x_time.reset();
                if(!isInsideClampUp){
                    dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_UP);
                    isInsideClampUp = true;
                }else if(isInsideClampUp){
                    dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_DOWN);
                    isInsideClampUp = false;
                }
            }

            if(gamepad2.b && b_time.seconds()>0.25){
                b_time.reset();
                if(!isOutsideClampUp){
                    dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_UP);
                    isOutsideClampUp = true;
                }else if(isOutsideClampUp){
                    dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_DOWN);
                    isOutsideClampUp = false;
                }
            }

            if(gamepad2.y && y_time.seconds()>0.25){
                y_time.reset();
                parkingArmServo.setPosition(PARKING_ARM_OUT);
            }
            if(gamepad2.dpad_right){
                dumperArmServo.setPosition(1);
            }

            if(gamepad2.right_bumper && right_bumper_time.seconds()>0.25){
                right_bumper_time.reset();
                if(Constants12907.DumperArmDropPositions.DOWN.equals(dumperArmDropPositions)){
                    dumperArmServo.setPosition(DUMPER_ARM_DROP_3);
                    dumperArmDropPositions = Constants12907.DumperArmDropPositions.DROP3;
                }else if(Constants12907.DumperArmDropPositions.DROP3.equals(dumperArmDropPositions)){
                    dumperArmServo.setPosition(DUMPER_ARM_DROP_2);
                    dumperArmDropPositions = Constants12907.DumperArmDropPositions.DROP2;
                }else if(Constants12907.DumperArmDropPositions.DROP2.equals(dumperArmDropPositions)){
                    dumperArmServo.setPosition(DUMPER_ARM_DROP_1);
                    dumperArmDropPositions = Constants12907.DumperArmDropPositions.DROP1;
                }else if(Constants12907.DumperArmDropPositions.DROP1.equals(dumperArmDropPositions)){
                    dumperArmServo.setPosition(DUMPER_ARM_DROP_3);
                    dumperArmDropPositions = Constants12907.DumperArmDropPositions.DROP3;
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
    private void newIntakeControl(){
        if(gamepad2.a && a_time.seconds()>0.25){
            a_time.reset();
            if(!isIntakeSpinning){
                leftIntakeMotor.setPower(-0.4);
                rightIntakeMotor.setPower(0.4);
                isIntakeSpinning = true;

            }else if( isIntakeSpinning){
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);
                isIntakeSpinning = false;
            }
            if(gamepad2.dpad_up){
                leftIntakeServo.setPosition(INTAKE_RIGHT_CLOSE);
            }
            if(gamepad2.dpad_down){
                leftIntakeServo.setPosition(INTAKE_RIGHT_OPEN);
            }

        }

    }
    private void dumperControl(){
        if(gamepad2.x && x_time.seconds()>0.25){
            x_time.reset();
            if(!isInsideClampUp){
                dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_UP);
                isInsideClampUp = true;
            }else if(isInsideClampUp){
                dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_DOWN);
                isInsideClampUp = false;
            }
        }

        if(gamepad2.b && b_time.seconds()>0.25){
            b_time.reset();
            if(!isOutsideClampUp){
                dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_UP);
                isOutsideClampUp = true;
            }else if(isOutsideClampUp){
                dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_DOWN);
                isOutsideClampUp = false;
            }
        }

        if(gamepad2.y && y_time.seconds()>0.25){
            y_time.reset();
            parkingArmServo.setPosition(PARKING_ARM_OUT);
        }
        if(gamepad2.dpad_right){
            dumperArmServo.setPosition(1);
        }

        if(gamepad2.right_bumper && right_bumper_time.seconds()>0.25){
            right_bumper_time.reset();
            if(Constants12907.DumperArmDropPositions.DOWN.equals(dumperArmDropPositions)){
                dumperArmServo.setPosition(DUMPER_ARM_DROP_3);
                dumperArmDropPositions = Constants12907.DumperArmDropPositions.DROP3;
            }else if(Constants12907.DumperArmDropPositions.DROP3.equals(dumperArmDropPositions)){
                dumperArmServo.setPosition(DUMPER_ARM_DROP_2);
                dumperArmDropPositions = Constants12907.DumperArmDropPositions.DROP2;
            }else if(Constants12907.DumperArmDropPositions.DROP2.equals(dumperArmDropPositions)){
                dumperArmServo.setPosition(DUMPER_ARM_DROP_1);
                dumperArmDropPositions = Constants12907.DumperArmDropPositions.DROP1;
            }else if(Constants12907.DumperArmDropPositions.DROP1.equals(dumperArmDropPositions)){
                dumperArmServo.setPosition(DUMPER_ARM_DROP_3);
                dumperArmDropPositions = Constants12907.DumperArmDropPositions.DROP3;
            }

        }

    }


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
        if(gamepad1.a && a_time.seconds() >= 0.25){
            a_time.reset();
            if(!isSideArmDown){
                sideArmServo.setPosition(SIDE_ARM_LOWERED);
                isSideArmDown = true;
            }
            else if(isSideArmDown){
                sideArmServo.setPosition(SIDE_ARM_RAISED);
                isSideArmDown = false;
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
        Thread attachments = new AttachmentsThread();
        waitForStart();
        attachments.start();
        while(opModeIsActive()){
            mecanumDrive(scaleFactor);
            if(gamepad1.x){
                scaleFactor=0.5;
            }
            if(gamepad1.y){
                scaleFactor=0.75;
            }
            repositioningControl();
            sideArmControl();
            // Code going into thread
            //newIntakeControl();
            //dumperControl();
            idle();
        }
          attachments.interrupt();
    }
}