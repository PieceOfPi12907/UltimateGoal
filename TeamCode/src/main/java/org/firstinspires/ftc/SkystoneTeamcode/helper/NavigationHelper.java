package org.firstinspires.ftc.SkystoneTeamcode.helper;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NavigationHelper {

    public void navigate (double pTgtDistance, Constants12907.Direction pDirection, double pRotation, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, Telemetry telemetry ){
        if(pDirection.equals(Constants12907.Direction.FWD)){
            forwardDrive(pTgtDistance, pSpeed, pBackLeft, pBackRight, pFrontRight, pFrontLeft, telemetry);
        }
        else if(pDirection.equals(Constants12907.Direction.BWD)){

        }
        else if(pDirection.equals(Constants12907.Direction.LEFT)){
            leftStrafe(pTgtDistance, pSpeed, pBackLeft, pBackRight, pFrontRight, pFrontLeft, telemetry);

        }
        else if(pDirection.equals(Constants12907.Direction.RIGHT)){

        }
        else if(pDirection.equals(Constants12907.Direction.TURN)){

        }

    }


    private void forwardDrive (double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, Telemetry telemetry) {
        ElapsedTime runtime = new ElapsedTime();

        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        int newTargetPositionRight;
        int newTargetPositionLeft;

        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(newTargetPositionLeft);
        pBackRight.setTargetPosition(newTargetPositionRight);
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pFrontRight.setPower((pSpeed));
        pBackRight.setPower((pSpeed));
        pFrontLeft.setPower((pSpeed));
        pBackLeft.setPower((pSpeed));

        while ((pBackLeft.isBusy() && pBackRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newTargetPositionLeft, newTargetPositionRight);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    pBackLeft.getCurrentPosition(),
                    pBackRight.getCurrentPosition());
            telemetry.update();
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                 pBackLeft.getCurrentPosition(),
                 pBackRight.getCurrentPosition());
        telemetry.update();
    }

    private void leftStrafe(double pTgtDistance, double pSpeed, DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, Telemetry telemetry) {
        ElapsedTime runtime = new ElapsedTime();

        final double COUNTS_PER_MOTOR_DCMOTOR = 1120;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DCMOTOR * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        int newTargetPositionRight;
        int newTargetPositionLeft;

        pBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Determine new target position, and pass to motor controller
        newTargetPositionLeft = pBackLeft.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        newTargetPositionRight = pBackRight.getCurrentPosition() + (int) (pTgtDistance * COUNTS_PER_INCH);
        pBackLeft.setTargetPosition(newTargetPositionLeft);
        pBackRight.setTargetPosition(-(newTargetPositionRight));
        runtime.reset();

        telemetry.addData("Initial Value", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(), pBackRight.getCurrentPosition());
        telemetry.update();

        pBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pFrontRight.setPower((pSpeed));
        pBackRight.setPower(-(pSpeed));
        pFrontLeft.setPower(-(pSpeed));
        pBackLeft.setPower((pSpeed));

        while ((pBackLeft.isBusy() && pBackRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newTargetPositionLeft, newTargetPositionRight);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    pBackLeft.getCurrentPosition(),
                    pBackRight.getCurrentPosition());
            telemetry.update();
        }

        //stop motors
        pFrontLeft.setPower(0);
        pFrontRight.setPower(0);
        pBackLeft.setPower(0);
        pBackRight.setPower(0);

        telemetry.addData("Final Position", "Running at %7d :%7d",
                pBackLeft.getCurrentPosition(),
                pBackRight.getCurrentPosition());
        telemetry.update();


    }

}


