package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Repositioning {
    int negative;

    double leftServoDown = 0.95;
    double rightServoDown = 0.01;

    double leftServoUp = 0.1;
    double rightServoUp = 0.85;


    /*public void doRepositioning(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue, Servo repositioningRight, Servo repositioningLeft) {
        if (isBlue == false) {
            negative = -1;
        } else {
            negative = 1;
        }

        //move straight along the wall to align with foundation
        pNavigate.navigate(14*negative, Constants12907.Direction.STRAIGHT,0,0.4*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        //strafe right to the foundation
        pNavigate.navigate(34, Constants12907.Direction.RIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        repositioningLeft.setPosition(leftServoDown);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningRight.setPosition(rightServoDown);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //strafe back to wall pulling foundation along
        //pNavigate.navigate(38, Constants12907.Direction.LEFT,0,0.3,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        leftStrafeWithoutCorrection(40,0.3, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        repositioningLeft.setPosition(leftServoUp);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningRight.setPosition(rightServoUp);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    public void doSimpleRepositioning(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue, Servo repositioningRight, Servo repositioningLeft) {

        repositioningLeft.setPosition(leftServoDown);
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningRight.setPosition(rightServoDown);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //strafe back to wall pulling foundation along
        //pNavigate.navigate(38, Constants12907.Direction.LEFT,0,0.3,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        if(isBlue != true){
            leftStrafeWithoutCorrection(43,0.3, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

        } else {
            leftStrafeWithoutCorrection(43,0.3, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        }

        repositioningLeft.setPosition(leftServoUp);
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningRight.setPosition(rightServoUp);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }*/


    public void doAngleRepositioning(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Boolean isBlue, Boolean isOuter, Boolean isStoneRepo, Servo repositioningRight, Servo repositioningLeft, ElapsedTime runtime) {
        if (isBlue == false) {
            negative = -1;
        } else {
            negative = 1;
        }

        if(isStoneRepo != true){
            pNavigate.navigate(10*negative, Constants12907.Direction.STRAIGHT,0,0.4*negative,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

            //pNavigate.navigate(38, Constants12907.Direction.RIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
            rightStrafeWithoutCorrection(38, 0.4, pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

            repositioningLeft.setPosition(leftServoDown);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            repositioningRight.setPosition(rightServoDown);
            try {
                Thread.sleep(750);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        //pNavigate.navigate(20, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        if(isBlue == true){
            if(isStoneRepo == true){
                leftStrafeWithoutCorrection(34, 0.4, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            } else {
                leftStrafeWithoutCorrection(30, 0.4, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            }

            turnWithEncoders(pFrontRight, pFrontLeft, pBackRight, pBackLeft, 90*negative, 0.5, pImu, pTelemetry);

            rightStrafeWithoutCorrection(35,0.4,pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry );
        }else{
            if(isStoneRepo == true){
                leftStrafeWithoutCorrection(34, 0.4, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            } else {
                leftStrafeWithoutCorrection(30, 0.4, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            }

            turnWithEncoders(pFrontRight, pFrontLeft, pBackRight, pBackLeft, 90*negative, 0.5, pImu, pTelemetry);

            rightStrafeWithoutCorrection(35,0.4,pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry );
        }

        repositioningLeft.setPosition(leftServoUp);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        repositioningRight.setPosition(rightServoUp);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(isStoneRepo == false || runtime.seconds()<=26 ) {
            if (isOuter == true) {
                pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //pNavigate.navigate(42, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

                //pNavigate.navigate(10, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);

                leftStrafeWithoutCorrection(5, 0.4, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.mode = BNO055IMU.SensorMode.IMU;
                pImu.initialize(parameters);

                //pNavigate.navigate(28, Constants12907.Direction.STRAIGHT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
                pNavigate.navigate(24 * negative, Constants12907.Direction.STRAIGHT, 0, 0.4 * negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

                //pNavigate.navigate(30, Constants12907.Direction.LEFT,0,0.4,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
                leftStrafeWithoutCorrection(40, 0.4, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);


            } else if (isOuter == false) {
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.mode = BNO055IMU.SensorMode.IMU;
                pImu.initialize(parameters);

                pNavigate.navigate(45, Constants12907.Direction.LEFT, 0, 0.4, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            }
        }
    }




    public void turnWithEncoders(DcMotor pFrontRight, DcMotor pFrontLeft, DcMotor pBackRight,
                                 DcMotor pBackLeft, double pRotation, double pSpeed, BNO055IMU pImu, Telemetry pTelemetry) {
        double computedAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        double currentAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        double previousAngle = 0.0;
        pTelemetry.addData("Initial Angle: ", computedAngle);
        pTelemetry.update();



        pFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (pRotation > 0) {
            while (computedAngle < pRotation) {
                //right
                previousAngle = computedAngle;
                pFrontRight.setPower(pSpeed);
                pBackRight.setPower(pSpeed);
                pFrontLeft.setPower(-pSpeed);
                pBackLeft.setPower(-pSpeed);
                computedAngle = ((pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES)).firstAngle) - currentAngle;
            }
        } else {
            while (computedAngle > pRotation) {
                //left
                previousAngle = computedAngle;
                pFrontRight.setPower(-pSpeed);
                pBackRight.setPower(-pSpeed);
                pFrontLeft.setPower(pSpeed);
                pBackLeft.setPower(pSpeed);
                computedAngle = ((pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        AngleUnit.DEGREES)).firstAngle) - currentAngle;
            }

        }
        pFrontRight.setPower(0);
        pBackRight.setPower(0);
        pFrontLeft.setPower(0);
        pBackLeft.setPower(0);
        currentAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;
        pTelemetry.addData("Previous Angle: ",previousAngle);
        pTelemetry.addData("Computed Angle: ",computedAngle);

        pTelemetry.addData("Current Angle: ",currentAngle);
        pTelemetry.update();
    }

    private void driveStraight (double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, Telemetry telemetry, BNO055IMU pImu) {
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
        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, converts it, and pass to motor controller
        newTargetPositionLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionLeft = pFrontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionRight = pFrontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(newTargetPositionLeft);
        pBackRight.setTargetPosition(newTargetPositionRight);
        pFrontLeft.setTargetPosition(newTargetPositionLeft);
        pFrontRight.setTargetPosition(newTargetPositionRight);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition());
        telemetry.addData("New Target Position","Left %7d : Right %7d", newTargetPositionLeft,newTargetPositionRight);
        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pFrontLeft.getCurrentPosition(), pFrontRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        pFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*pFrontRight.setPower((pSpeed));
        pBackRight.setPower((pSpeed));
        pFrontLeft.setPower((pSpeed));
        pBackLeft.setPower((pSpeed));*/

        double correction;

        //This while loop will keep the motors running to the target position until one of the motors have reached the final encoder count
        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {
            correction = 0;
            pFrontRight.setPower(pSpeed+correction);
            pBackRight.setPower(pSpeed+correction);
            pFrontLeft.setPower(pSpeed-correction);
            pBackLeft.setPower(pSpeed-correction);
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d : %7d: %7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition(),
                pFrontLeft.getCurrentPosition(),
                pFrontRight.getCurrentPosition());
        telemetry.update();
    }

    private void leftStrafeWithoutCorrection(double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft,  BNO055IMU pImu, Telemetry telemetry) {
        ElapsedTime runtime = new ElapsedTime();

        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        int newTargetPositionFrontRight;
        int newTargetPositionFrontLeft;
        int newTargetPositionBackRight;
        int newTargetPositionBackLeft;

        double startingAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionBackRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontLeft = pFrontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontRight = pFrontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(newTargetPositionBackLeft);
        pBackRight.setTargetPosition(-(newTargetPositionBackRight));
        pFrontRight.setTargetPosition(newTargetPositionFrontRight);
        pFrontLeft.setTargetPosition(-(newTargetPositionBackLeft));
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition(), pFrontLeft.getCurrentPosition(), pFrontRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pFrontRight.setPower((pSpeed));
        pBackRight.setPower(-(pSpeed));
        pFrontLeft.setPower(-(pSpeed));
        pBackLeft.setPower((pSpeed));

        telemetry.addData("Path1", "Target Position %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);

        telemetry.update();

        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {

            // Display it for the driver.

        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition(),
                pFrontLeft.getCurrentPosition(),
                pFrontRight.getCurrentPosition());
        telemetry.update();
    }

    private void rightStrafeWithoutCorrection(double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft,  BNO055IMU pImu, Telemetry telemetry) {
        ElapsedTime runtime = new ElapsedTime();

        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        int newTargetPositionFrontRight;
        int newTargetPositionFrontLeft;
        int newTargetPositionBackRight;
        int newTargetPositionBackLeft;

        double startingAngle = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionBackLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionBackRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontLeft = pFrontLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionFrontRight = pFrontRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(-(newTargetPositionBackLeft));
        pBackRight.setTargetPosition(newTargetPositionBackRight);
        pFrontRight.setTargetPosition(-(newTargetPositionFrontRight));
        pFrontLeft.setTargetPosition(newTargetPositionBackLeft);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition(), pFrontLeft.getCurrentPosition(), pFrontRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pFrontRight.setPower(-(pSpeed));
        pBackRight.setPower((pSpeed));
        pFrontLeft.setPower((pSpeed));
        pBackLeft.setPower(-(pSpeed));

        telemetry.addData("Path1", "Target Position %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);
        telemetry.update();

        while ((pBackLeft.isBusy() && pBackRight.isBusy() && pFrontLeft.isBusy() && pFrontRight.isBusy())) {

            // Display it for the driver.
            /*telemetry.addData("Path1", "Running to %7d :%7d", newTargetPositionBackLeft, newTargetPositionBackRight, newTargetPositionFrontLeft, newTargetPositionFrontRight);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    pBackLeft.getCurrentPosition(),
                    pBackRight.getCurrentPosition(),
                    pFrontLeft.getCurrentPosition(),
                    pFrontRight.getCurrentPosition());
            telemetry.update();*/
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition(),
                pFrontLeft.getCurrentPosition(),
                pFrontRight.getCurrentPosition());
        telemetry.update();
    }
}


