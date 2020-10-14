package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@Autonomous (name = "RampTester" ,group = "autonomous")
public class RampUpRampDownTester extends LinearOpMode{
        NavigationHelper navigateTest = new NavigationHelper();
        DcMotor frontLeft;
        DcMotor backLeft;
        DcMotor frontRight;
        DcMotor backRight;
        BNO055IMU imu;
        PIDController pidDrive;

        Servo sideClampServo;
        Servo sideArmServo;
        Servo leftIntakeServo;
        Servo rightIntakeServo;

        final double INTAKE_RIGHT_CLOSE = 0.84;
        final double INTAKE_LEFT_CLOSE = 0.25;

    Orientation lastAngles = new Orientation();
    double globalAngle;
        public void initialize() {
            //Configuration of the motors
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            sideClampServo = hardwareMap.get(Servo.class, "blockClamper");
            rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
            leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
            sideArmServo = hardwareMap.get(Servo.class, "pivotGrabber");

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

             pidDrive = new PIDController(.05, 0, 0);

            leftIntakeServo.setPosition(INTAKE_LEFT_CLOSE);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            rightIntakeServo.setPosition(INTAKE_RIGHT_CLOSE);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            sideClampServo.setPosition(0.3);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            sideArmServo.setPosition(0.4);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        /*frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/
        }

        @Override
        public void runOpMode() throws InterruptedException {

            try {

                initialize();

                while (!isStopRequested() && !imu.isGyroCalibrated()) {
                    sleep(50);
                    idle();
                }
                telemetry.addData("imu calibration status: ", imu.getCalibrationStatus().toString());
                telemetry.update();

                waitForStart();

                if (opModeIsActive()) {
                    double imu_correct = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


                    sideArmServo.setPosition(0.8);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideClampServo.setPosition(1);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideArmServo.setPosition(0.3);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    forwardDrive(90, 1);


                    sideArmServo.setPosition(0.8);
                    try {
                        Thread.sleep(25);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideClampServo.setPosition(0);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideArmServo.setPosition(0.4);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideClampServo.setPosition(1);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }


                    double imuCorrection = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
                    telemetry.addData("IMU CORRECTION: ", imuCorrection);
                    telemetry.update();

                    forwardDrive(-90, -1);


                    imuCorrection = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
                    telemetry.addData("IMU CORRECTION: ", imuCorrection);
                    telemetry.update();


                    sideClampServo.setPosition(0.3);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideArmServo.setPosition(0.8);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideClampServo.setPosition(1);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideArmServo.setPosition(0.3);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    forwardDrive(90, 1);


                    imuCorrection = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
                    telemetry.addData("IMU CORRECTION: ", imuCorrection);
                    telemetry.update();
                    
                    navigateTest.navigate(10, Constants12907.Direction.RIGHT, 0, 0.5, backLeft, backRight, frontRight, frontLeft, imu, telemetry, false);

                    sideArmServo.setPosition(0.8);
                    try {
                        Thread.sleep(25);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideClampServo.setPosition(0);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideArmServo.setPosition(0.4);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    sideClampServo.setPosition(1);
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }


                }
            } catch (Exception bad){
                telemetry.addData("EXCEPTION!!!:", bad.getMessage());
                bad.printStackTrace();
                telemetry.update();
            }
        }

        private void forwardDrive (double pTgtDistance, int direction) {
            ElapsedTime runtime = new ElapsedTime();

            //Variables used for converting inches to Encoder dounts
            final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
            final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
            final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
            final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);
            // newTargetPosition is the target position after it has been converted
            int newTargetPositionRight;
            int newTargetPositionLeft;
            // Sets all encoder values to 0 as we are moving robot with all 4 encoders
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, converts it, and pass to motor controller
            newTargetPositionLeft = backLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
            newTargetPositionRight = backRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
            newTargetPositionLeft = frontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
            newTargetPositionRight = frontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
            backLeft.setTargetPosition(newTargetPositionLeft);
            backRight.setTargetPosition(newTargetPositionRight);
            frontLeft.setTargetPosition(newTargetPositionLeft);
            frontRight.setTargetPosition(newTargetPositionRight);
            runtime.reset();

            telemetry.addData("Initial Value", "Running at %7d :%7d",
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            telemetry.addData("New Target Position","Left %7d : Right %7d", newTargetPositionLeft,newTargetPositionRight);
            telemetry.addData("Initial Value", "Running at %7d :%7d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();

            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    /*frontRight.setPower((pSpeed));
    backRight.setPower((pSpeed));
    frontLeft.setPower((pSpeed));
    backLeft.setPower((pSpeed));*/

            //This while loop will keep the motors running to the target position until one of the motors have reached the final encoder count
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0,0.5);
            pidDrive.setInputRange(90, 90);
            pidDrive.enable();

            int count = 0;

            while ((backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy() && opModeIsActive()) || isStopRequested()) {

                double power = direction * setRampPower(newTargetPositionRight);

                double correction = pidDrive.performPID(getAngle(imu));

                if(isStopRequested()){
                    frontRight.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    break;
                } else{
                    frontRight.setPower(power + correction);
                    backRight.setPower(power + correction);
                    frontLeft.setPower(power - correction);
                    backLeft.setPower(power - correction);
                }

            }

            //stop motors
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            telemetry.addData("Final Position", "Running at %7d :%7d : %7d: %7d",
                    backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition());
            telemetry.update();
        }

            public double setRampPower (int totalEncoders) {
                double rampUp = Math.abs(totalEncoders / 10);
                double constantSpeed = Math.abs((totalEncoders / 10) * 6);
                double rampDown = Math.abs((totalEncoders/ 10)*3);
                double power;

                if (Math.abs(backRight.getCurrentPosition())<rampUp) {
                     //power = 0.5 + ((backRight.getCurrentPosition() / rampUp)/2);
                    power = 0.75;
                }
                else if(Math.abs(backRight.getCurrentPosition())>(Math.abs(totalEncoders)-rampDown)){
                     //power = 1.0 - (0.8*(((backRight.getCurrentPosition()-(rampUp+constantSpeed))/rampDown)));
                    power = 0.4;
                     telemetry.addLine("PHASE 3");
                     telemetry.update();
                }
                else{
                     power = 1.0;
                }
                return power;
            }


    private double getAngle(BNO055IMU pImu){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
            }
