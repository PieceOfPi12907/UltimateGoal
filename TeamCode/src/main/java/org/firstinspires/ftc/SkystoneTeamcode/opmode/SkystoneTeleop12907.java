package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name= "FINAL teleop 12907", group = "teleop")
public class SkystoneTeleop12907 extends LinearOpMode {
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor intakeLeft;
    DcMotor intakeRight;
    Servo autoArm;
    Servo autoClamper;
    Servo dumperArm;
    Servo dumperClamp;
    Servo dumperRotate;
    Servo leftIntake;
    Servo rightIntake;
    Servo repositioning;



    boolean threadStarted = false;
    boolean isClamped = false;
    boolean isArmDown = false;
    boolean isArmStraight = false;
    boolean isAutoArmDown = false;
    boolean isAutoArmClamped = false;
    double scaleFactor = 0.7;
    double intakeSpeed = 0;

    final double AUTO_ARM_LOWERED = 0.9;
    final double AUTO_ARM_RAISED = 0.4;
    final double AUTO_CLAMP_OPENED = 0.5;
    final double AUTO_CLAMP_CLOSED = 0.8;
    final double INTAKE_LEFT_CLOSE = 0.71;
    final double INTAKE_RIGHT_CLOSE = 0.8;
    // final double INTAKE_LEFT_OPEN = 0;
    //final double INTAKE_RIGHT_OPEN = 0;
    final double DUMPER_ARM_OUT_FIRST = 0.4;
    final double DUMPER_ARM_OUT_SECOND = 0.68;
    final double DUMPER_ARM_IN = 1;
    final double DUMPER_CLAMP_DOWN = 0.9;
    final double DUMPER_CLAMP_UP = 0.3;
    final double DUMPER_ROTATE_IN = 0.8;
    final double DUMPER_ROTATE_OUT = 0;
    final double REPOSITIONING_DOWN = 0.75;
    final double REPOSITIONING_UP = 0.15;
    boolean goingReverse = false;
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime a_time = new ElapsedTime();




    public void initialize(){
        frontRight=hardwareMap.get(DcMotor.class,"frontRightMotor");
        frontLeft=hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRight=hardwareMap.get(DcMotor.class,"backRightMotor");
        backLeft=hardwareMap.get(DcMotor.class,"backLeftMotor");
        //pivotGrabber=hardwareMap.get(Servo.class, "pivotGrabber");
        //blockClamper=hardwareMap.get(Servo.class, "blockClamper");

        intakeLeft = hardwareMap.get(DcMotor.class,"leftIntakeMotor");
        intakeRight = hardwareMap.get(DcMotor.class,"rightIntakeMotor");

        rightIntake = hardwareMap.get(Servo.class, "rightIntakeServo");
        leftIntake = hardwareMap.get(Servo.class, "leftIntakeServo");
        dumperArm = hardwareMap.get(Servo.class,"dumperArmServo");
        dumperClamp = hardwareMap.get(Servo.class,"dumperClampServo");
        dumperRotate = hardwareMap.get(Servo.class,"dumperPivotServo");
        repositioning = hardwareMap.get(Servo.class, "repositioningServo");
        autoArm = hardwareMap.get(Servo.class, "pivotGrabber");
        autoClamper = hardwareMap.get(Servo.class, "blockClamper");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntake.setPosition(INTAKE_LEFT_CLOSE);
        rightIntake.setPosition(INTAKE_RIGHT_CLOSE);
        // dumperArmServo.setPosition(DUMPER_ARM_IN);
        dumperClamp.setPosition(DUMPER_CLAMP_UP);
        repositioning.setPosition(REPOSITIONING_UP);

        //blockClamper.setPosition(CLAMP_CLOSED);

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

    public void deadZoneTest (double scale){
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
        frontLeft.setPower((fLPower) * scale);
        backLeft.setPower((bLPower) * scale);
        frontRight.setPower((fRPower) * scale);
        backRight.setPower((bRPower) * scale);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        class test extends Thread{
            public void run() {
                while(true) {
                    intakeLeft.setPower(intakeSpeed);
                    intakeRight.setPower(-intakeSpeed);
                    if(gamepad2.a){
                        intakeLeft.setPower(0);
                        intakeRight.setPower(0);
                        threadStarted=false;
                        break;
                    }
                    if(gamepad2.right_trigger>0){
                        intakeRight.setPower(intakeSpeed);
                        intakeLeft.setPower(-intakeSpeed);
                        goingReverse = true;
                    }
                    if(goingReverse && gamepad2.right_trigger==0){
                        intakeLeft.setPower(0);
                        intakeRight.setPower(0);
                        threadStarted=false;
                        break;
                    }

                }
            }
        }
        Thread ourDrive = new test();
        b_time.reset();
        y_time.reset();
        while(opModeIsActive()){
            deadZoneTest(scaleFactor);
            if(gamepad1.x){
                scaleFactor=0.5;
            }
            if(gamepad1.y){
                scaleFactor=0.75;
            }
            if(gamepad2.a){
                /*if(threadStarted==false){
                    ourDrive.run();
                }

                 */
                intakeLeft.setPower(-0.4);
                intakeRight.setPower(0.4);
            }
            if(gamepad2.left_bumper){
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                rightIntake.setPosition(0.50);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                dumperClamp.setPosition(DUMPER_CLAMP_DOWN);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                // rightIntakeServo.setPosition(INTAKE_RIGHT_OPEN);

            }
            if(gamepad2.right_bumper){
                leftIntake.setPosition(INTAKE_LEFT_CLOSE);
                rightIntake.setPosition(INTAKE_RIGHT_CLOSE);
            }
            if(gamepad2.b&&b_time.seconds()>=0.5){
                b_time.reset();
                if(!isClamped){
                    dumperClamp.setPosition(DUMPER_CLAMP_DOWN);
                    isClamped = true;
                }
                else if(isClamped){
                    dumperClamp.setPosition(DUMPER_CLAMP_UP);
                    isClamped = false;
                }
            }
            if(gamepad2.y&&y_time.seconds()>=0.5){
                y_time.reset();
                if(!isArmDown){
                    dumperArm.setPosition(DUMPER_ARM_IN);
                    isArmDown = true;
                }
                else if(isArmDown){
                    dumperArm.setPosition(DUMPER_ARM_OUT_FIRST);
                    isArmDown = false;
                }
            }
            if(gamepad2.x && x_time.seconds() >=0.5){
                x_time.reset();
                if(!isArmStraight){
                    dumperRotate.setPosition(DUMPER_ROTATE_OUT);
                    isArmStraight = true;
                }
                else if(isArmStraight){
                    dumperRotate.setPosition(DUMPER_ROTATE_IN);
                    isArmStraight = false;
                }
            }
            if(gamepad2.dpad_up){
                dumperArm.setPosition(DUMPER_ARM_OUT_SECOND);
            }
            if(gamepad2.dpad_down){
                dumperArm.setPosition(DUMPER_ARM_IN);
            }
            if(gamepad1.right_bumper){
                repositioning.setPosition(REPOSITIONING_DOWN);
            }
            if(gamepad1.left_bumper){
                repositioning.setPosition(REPOSITIONING_UP);
            }


            if(gamepad1.a && a_time.seconds() >= 0.5){
                a_time.reset();
                if(!isAutoArmDown){
                    autoArm.setPosition(AUTO_ARM_LOWERED);
                    isAutoArmDown = true;
                }
                else if(isAutoArmDown){
                    autoArm.setPosition(AUTO_ARM_RAISED);
                    isAutoArmDown = false;
                }
            }
            if(gamepad1.b && b_time.seconds() >= 0.5){
                b_time.reset();
                if(!isAutoArmClamped){
                    autoClamper.setPosition(AUTO_CLAMP_CLOSED);
                    isAutoArmClamped = true;
                }
                else if(isAutoArmClamped){
                    autoClamper.setPosition(AUTO_CLAMP_OPENED);
                    isAutoArmClamped = false;
                }
            }



           /*
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
            */

            /*frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            */


        }
    }
}
