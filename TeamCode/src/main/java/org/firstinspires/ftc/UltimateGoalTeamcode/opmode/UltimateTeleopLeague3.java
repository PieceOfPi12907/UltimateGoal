package org.firstinspires.ftc.UltimateGoalTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.UltimateGoalTeamcode.helper.Constants2020;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.NavigationHelper;

@TeleOp(name = "FINAL ULTIMATE TELEOP LEAGUE 3",group = "teleop")
public class UltimateTeleopLeague3 extends LinearOpMode {

    BNO055IMU imu;

    DcMotor intake;
    Servo shooterIntakeServo;
    Servo wing;
    Servo umbrella;
    DcMotor shooter;

    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor wobbleHingeMotor;
    Servo wobbleHingeServo;
    Servo wobbleClampServo;
    Constants2020.HingeServoPositions hingeServoPos = Constants2020.HingeServoPositions.UP;

    boolean isClamped = true;
    boolean isDown = true;
    double scaleFactor = 0.999;
    boolean shooterIntakeSpinning = false;
    boolean shooterIntakeInit = true;
    boolean intakeOn = false;
    boolean intakeBack = false;
    boolean outtakeBack = false;
    boolean wingUp = false;
    boolean hingeMotorDown = false;
    public final double SHOOTER_INTAKE_SERVO_INIT = 0.54;
    public final double SHOOTER_INTAKE_SERVO_OPEN = 0.50;
    public final double SHOOTER_INTAKE_SERVO_CLOSE = 0.05;

    //hinge and clamp values to be tested:
    double currentpos = 0;
    final double HINGE_SERVO_UP = 0.95; //outside to grab wobble goal
    final double HINGE_SERVO_MID = 0.65;
    final double HINGE_SERVO_DOWN = 0.01; //inside the robot
    final double CLAMP_SERVO_IN = 0.7; //0.6 clamp
    final double CLAMP_SERVO_OUT = 0.2;
    final double WING_INIT = 0.99;
    final double WING_DOWN = 0.6;
    double shooterSpeed = 0.8;
    double currentPos = 0.5;

    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime rb_time = new ElapsedTime();
    ElapsedTime right_bumper_time = new ElapsedTime();
    ElapsedTime left_bumper_time = new ElapsedTime();
    ElapsedTime g1_a_time = new ElapsedTime();
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime dpad_time = new ElapsedTime();
    ElapsedTime right_stick_time = new ElapsedTime();

    private void initialize() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterIntakeServo = hardwareMap.get(Servo.class, "shooterIntakeServo");
        shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_INIT);
        wing = hardwareMap.get(Servo.class,"wing");
        wing.setPosition(WING_INIT);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");

        umbrella = hardwareMap.get(Servo.class, "umbrella");
        wobbleClampServo = hardwareMap.get(Servo.class, "clamp");
        wobbleHingeMotor = hardwareMap.get(DcMotor.class,"wobbleHingeMotor");
        wobbleHingeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        hingeServoPos = Constants2020.HingeServoPositions.UP;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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
                    powershotControl();
                    idle();
                }
            } catch (Exception e) {

            }
        }

        private void powershotControl(){
            if(gamepad2.a){
                shooter.setPower(-shooterSpeed);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIntakeServo.setPosition(0.42);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                /*
                frontLeftMotor.setPower(0.5);
                frontRightMotor.setPower(-0.5);
                backLeftMotor.setPower(-0.5);
                backRightMotor.setPower(0.5);

                 */
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                shooterIntakeServo.setPosition(0.27);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                /*
                frontLeftMotor.setPower(0.5);
                frontRightMotor.setPower(-0.5);
                backLeftMotor.setPower(-0.5);
                backRightMotor.setPower(0.5);

                 */

                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);

                shooterIntakeServo.setPosition(0.07);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                shooterIntakeServo.setPosition(SHOOTER_INTAKE_SERVO_OPEN);
                shooter.setPower(0);

            }
        }
        private void intakeControl() {
            if (gamepad1.x && x_time.seconds() >= 0.25) {
                x_time.reset();
                if (shooterIntakeSpinning) {
                    telemetry.addLine("shooter off");
                    telemetry.update();
                    shooterIntakeSpinning = false;
                    intake.setPower(0);
                } else {
                    telemetry.addLine("shooter on");
                    telemetry.update();
                    shooterIntakeSpinning = true;
                    intake.setPower(0.99);
                }
            }


            if (gamepad1.y && y_time.seconds() >= 0.25) {
                y_time.reset();
                if (outtakeBack) {
                    intake.setPower(0);
                    outtakeBack = false;
                } else {
                    intake.setPower(-0.85);
                    outtakeBack = true;
                }
            }
            if (gamepad2.right_bumper && right_bumper_time.seconds()>=0.25) {
                right_bumper_time.reset();
                if (wingUp) {
                    wing.setPosition(WING_DOWN);
                    wingUp = false;
                } else {
                    wing.setPosition(WING_INIT);
                    wingUp = true;
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
                currentPos-=0.18;
                shooterIntakeServo.setPosition(currentPos);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            if (gamepad1.dpad_right && dpad_time.seconds() > 0.1) {
                dpad_time.reset();
                shooterSpeed = 0.63;
            }


            if (gamepad1.a && a_time.seconds() >= 0.25) {
                a_time.reset();
                if (intakeBack) {
                    shooter.setPower(0.1);
                    intakeBack = false;
                } else {
                    shooter.setPower(-shooterSpeed);
                    intakeBack = true;
                }
            }


            if(gamepad1.b && b_time.seconds() >= 0.25){
                b_time.reset();
                currentPos = 0.54;
                shooterIntakeServo.setPosition(currentPos);
                telemetry.addData("position", currentPos);
                telemetry.update();
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                currentPos-=0.15;
                shooterIntakeServo.setPosition(currentPos);
                telemetry.addData("position",currentPos);
                try {
                    sleep(600);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                currentPos-=0.15;
                shooterIntakeServo.setPosition(currentPos);
                try {
                    sleep(600);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                currentPos-=0.2;
                shooterIntakeServo.setPosition(currentPos);
                try {
                    sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                currentPos = 0.54;
                shooterIntakeServo.setPosition(currentPos);
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        private void wobbleArmControl() {
            /*if (gamepad1.right_bumper && right_bumper_time.seconds() > 0.25) {
                right_bumper_time.reset();
                if (hingeMotorDown) {
                    wobbleHingeMotor.setPower(-0.3);
                    hingeMotorDown = false;
                } else {
                    wobbleHingeMotor.setPower(0.3);
                    hingeMotorDown = true;
                }*/
            if (gamepad1.right_stick_y > 0 && right_stick_time.seconds() > 0.25) {
                right_stick_time.reset();
                wobbleHingeMotor.setPower(-0.85);
            } else if (gamepad1.right_stick_y < 0 && right_stick_time.seconds() > 0.25){
                right_stick_time.reset();
                wobbleHingeMotor.setPower(0.45);
            }
            else if (gamepad1.right_stick_y == 0 && right_stick_time.seconds() > 0.25){
                right_stick_time.reset();
                wobbleHingeMotor.setPower(0);
            }

                /*if (Constants2020.HingeServoPositions.UP.equals(hingeServoPos)) {
                    wobbleHingeServo.setPosition(HINGE_SERVO_DOWN);
                    hingeServoPos = Constants2020.HingeServoPositions.MID;
                } else if (Constants2020.HingeServoPositions.MID.equals(hingeServoPos)) {
                    wobbleHingeServo.setPosition(HINGE_SERVO_MID);
                    hingeServoPos = Constants2020.HingeServoPositions.DOWN;
                } else if (Constants2020.HingeServoPositions.DOWN.equals(hingeServoPos)) {
                    wobbleHingeServo.setPosition(HINGE_SERVO_UP);
                    hingeServoPos = Constants2020.HingeServoPositions.UP;
                }*/

            if (gamepad1.left_bumper && left_bumper_time.seconds() > 0.25) {
                left_bumper_time.reset();
                if (!isClamped) {
                    wobbleClampServo.setPosition(CLAMP_SERVO_OUT);
                    isClamped = true;
                    telemetry.addLine("SERVO IS NOT CLAMPED");
                    telemetry.update();
                } else if (isClamped) {
                    wobbleClampServo.setPosition(CLAMP_SERVO_IN);
                    isClamped = false;
                    telemetry.addLine("SERVO IS CLAMPED");
                    telemetry.update();
                }
            }

            if (gamepad2.left_bumper && g1_a_time.seconds() > 0.25) {
                g1_a_time.reset();
                if (!isDown) {
                    umbrella.setPosition(0);
                    isDown = true;
                    telemetry.addLine("position: " + umbrella.getPosition());
                    telemetry.update();
                } else if (isDown) {
                    umbrella.setPosition(0.5);
                    isDown = false;
                    telemetry.addLine("position: " + umbrella.getPosition());
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
        Thread attachments = new UltimateTeleopLeague3.AttachmentsThread();
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


            idle();
        }
        attachments.interrupt();
    }
}