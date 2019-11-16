package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "FINAL thread teleop 12907", group = "teleop")
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
    //Servo dumperClampServo;
   // Servo dumperPivotServo;
    Servo leftIntakeServo;
    Servo rightIntakeServo;
    Servo leftRepositionServo;
    Servo rightRepositionServo;


    boolean threadStarted = false;
    boolean isClamped = false;
    boolean isArmDown = false;
    boolean isArmStraight = false;
    boolean isSideArmDown = false;
    boolean isAutoArmClamped = false;

    boolean isIntakeSpinning = false;
    boolean isIntakeServoOpen = true;

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
    final double DUMPER_ARM_OUT_FIRST = 0.4;
    final double DUMPER_ARM_OUT_SECOND = 0.68;
    final double DUMPER_ARM_IN = 1;
    final double DUMPER_CLAMP_DOWN = 0.9;
    final double DUMPER_CLAMP_UP = 0.3;
    final double DUMPER_ROTATE_IN = 0.8;
    final double DUMPER_ROTATE_OUT = 0;
    final double LEFT_REPOSITIONING_DOWN = 1;
    final double LEFT_REPOSITIONING_UP = 0.25;
    final double RIGHT_REPOSITIONING_DOWN = 0.89;
    final double RIGHT_REPOSITIONING_UP = 0.36;

    boolean goingReverse = false;
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime a_time = new ElapsedTime();




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
        //dumperClampServo = hardwareMap.get(Servo.class,"dumperClampServo");
     //   dumperPivotServo = hardwareMap.get(Servo.class,"dumperPivot");
        leftRepositionServo = hardwareMap.get(Servo.class, "leftRepositioningServo");
        rightRepositionServo = hardwareMap.get(Servo.class,"rightRepositioningServo");
        sideArmServo = hardwareMap.get(Servo.class, "pivotGrabber");
        sideClampServo = hardwareMap.get(Servo.class, "blockClamper");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntakeServo.setPosition(INTAKE_LEFT_OPEN);
        rightIntakeServo.setPosition(INTAKE_RIGHT_OPEN);
        //dumperClampServo.setPosition(DUMPER_CLAMP_UP);
        leftRepositionServo.setPosition(LEFT_REPOSITIONING_UP);

    }



    private class AttachmentsThread extends Thread{
        boolean isIntakeSpinning = false;
        boolean isIntakeServoOpen = true;

        public void AttachmentsThread(){
            this.setName("Attachments Thread");
        }
        @Override
        public void run(){
            try {
                while(!isInterrupted()) {
                    intakeControl();
                    dumperControl();
                    idle();
                }
            }catch(Exception e){

            }
        }

        /*
           GAMEPAD2.a
         */
        private void intakeControl(){
            if(gamepad2.a){
                if(isIntakeServoOpen && !isIntakeSpinning){
                    leftIntakeMotor.setPower(-0.4);
                    rightIntakeMotor.setPower(0.4);
                    isIntakeSpinning = true;
                    telemetry.addLine("Intake is Spinning");
                }else if(isIntakeServoOpen && isIntakeSpinning){
                    leftIntakeMotor.setPower(0);
                    rightIntakeMotor.setPower(0);
                    rightIntakeServo.setPosition(INTAKE_RIGHT_CLOSE);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    //dumperClampServo.setPosition(DUMPER_CLAMP_DOWN);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    isIntakeSpinning = false;
                    isIntakeServoOpen = false;
                    telemetry.addLine("Intake stopped spinning and intake servo is closed");
                }else if(!isIntakeServoOpen && !isIntakeSpinning){
                    leftIntakeServo.setPosition(INTAKE_LEFT_OPEN);
                    rightIntakeServo.setPosition(INTAKE_RIGHT_OPEN);
                    isIntakeServoOpen = true;
                    isIntakeSpinning = false;
                    telemetry.addLine("Intake servo is open and Intake is not spinning");
                }else{
                    telemetry.addLine("Intake Servo and Motor error");
                }
                telemetry.update();

            }

        }
        private void dumperControl(){
            /*if(gamepad2.b&&b_time.seconds()>=0.25){
                b_time.reset();
                if(!isClamped){
                    dumperClampServo.setPosition(DUMPER_CLAMP_DOWN);
                    isClamped = true;
                }
                else if(isClamped){
                    dumperClampServo.setPosition(DUMPER_CLAMP_UP);
                    isClamped = false;
                }
            }
            */
            if(gamepad2.y&&y_time.seconds()>=0.25){
                y_time.reset();
                if(isArmDown){
                    dumperArmServo.setPosition(DUMPER_ARM_IN);
                    isArmDown = true;
                }
                else if(!isArmDown){
                    dumperArmServo.setPosition(DUMPER_ARM_OUT_FIRST);
                    isArmDown = false;

                }
            }
            if(gamepad2.x && x_time.seconds() >=0.25){
                x_time.reset();
                if(!isArmStraight){
       //             dumperPivotServo.setPosition(DUMPER_ROTATE_OUT);
                    isArmStraight = true;
                }
                else if(isArmStraight){
         //           dumperPivotServo.setPosition(DUMPER_ROTATE_IN);
                    isArmStraight = false;
                }
            }
            if(gamepad2.dpad_up){
                dumperArmServo.setPosition(DUMPER_ARM_OUT_SECOND);
            }
            if(gamepad2.dpad_down){
                dumperArmServo.setPosition(DUMPER_ARM_IN);
            }


        }

    } //end of thread class


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
    private void intakeControl(){
        if(gamepad2.a){
            if(isIntakeServoOpen && !isIntakeSpinning){
                leftIntakeMotor.setPower(-0.4);
                rightIntakeMotor.setPower(0.4);
                isIntakeSpinning = true;
            }else if(isIntakeServoOpen && isIntakeSpinning){
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);
                rightIntakeServo.setPosition(INTAKE_RIGHT_CLOSE);
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                //dumperClampServo.setPosition(DUMPER_CLAMP_DOWN);
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                isIntakeSpinning = false;
                isIntakeServoOpen = false;
            }else if(!isIntakeServoOpen && !isIntakeSpinning){
                leftIntakeServo.setPosition(INTAKE_LEFT_OPEN);
                rightIntakeServo.setPosition(INTAKE_RIGHT_OPEN);
                isIntakeServoOpen = true;
                isIntakeSpinning = false;
            }

        }

    }
    private void dumperControl(){
            /*if(gamepad2.b&&b_time.seconds()>=0.25){
                b_time.reset();
                if(!isClamped){
                    dumperClampServo.setPosition(DUMPER_CLAMP_DOWN);
                    isClamped = true;
                }
                else if(isClamped){
                    dumperClampServo.setPosition(DUMPER_CLAMP_UP);
                    isClamped = false;
                }
            }
            */
        if(gamepad2.y&&y_time.seconds()>=0.25){
            y_time.reset();
            if(isArmDown){
                dumperArmServo.setPosition(DUMPER_ARM_IN);
                isArmDown = true;
            }
            else if(!isArmDown){
                dumperArmServo.setPosition(DUMPER_ARM_OUT_FIRST);
                isArmDown = false;

            }
        }
        if(gamepad2.x && x_time.seconds() >=0.25){
            x_time.reset();
            if(!isArmStraight){
                //             dumperPivotServo.setPosition(DUMPER_ROTATE_OUT);
                isArmStraight = true;
            }
            else if(isArmStraight){
                //           dumperPivotServo.setPosition(DUMPER_ROTATE_IN);
                isArmStraight = false;
            }
        }
        if(gamepad2.dpad_up){
            dumperArmServo.setPosition(DUMPER_ARM_OUT_SECOND);
        }
        if(gamepad2.dpad_down){
            dumperArmServo.setPosition(DUMPER_ARM_IN);
        }


    }

    private void repositioning(){
        if(gamepad1.right_bumper){
            leftRepositionServo.setPosition(LEFT_REPOSITIONING_DOWN);
            rightRepositionServo.setPosition(RIGHT_REPOSITIONING_DOWN);
        }
        if(gamepad1.left_bumper){
            leftRepositionServo.setPosition(LEFT_REPOSITIONING_UP);
            rightRepositionServo.setPosition(RIGHT_REPOSITIONING_UP);
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
            if(!isAutoArmClamped){
                sideClampServo.setPosition(AUTO_CLAMP_CLOSED);
                isAutoArmClamped = true;
            }
            else if(isAutoArmClamped){
                sideClampServo.setPosition(AUTO_CLAMP_OPENED);
                isAutoArmClamped = false;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        Thread attachments = new AttachmentsThread();
        waitForStart();
        //attachments.start();
        while(opModeIsActive()){
            mecanumDrive(scaleFactor);
            if(gamepad1.x){
                scaleFactor=0.5;
            }
            if(gamepad1.y){
                scaleFactor=0.75;
            }
            repositioning();
            sideArmControl();
            // Code going into thread
            intakeControl();
            dumperControl();
            idle();
        }
        attachments.interrupt();
    }
}
