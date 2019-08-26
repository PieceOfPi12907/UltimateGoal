package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.MotorHelper;
import org.firstinspires.ftc.teamcode.helper.SensorHelper;

/**
 * Created by Twins on 11/17/18.
 */

public class SamplingBlue {

    public void forward(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, MotorHelper motorHelper, SensorHelper sensorHelper,
                        Telemetry telemetry, ColorSensor middleColor, DistanceSensor distance, Servo rightArm, Servo leftArm, Servo rightKnocker,
                        Servo leftKnocker, ColorSensor knockerColor) {

        double powerRight = 0.25;
        double powerLeft = 0.25;
        double targetPositionLeft = 18.5;
        double targetPositionRight = 18.5;
        double timeoutS = 5;
        motorHelper.movingWithEncoders(frontRight,frontLeft,backRight,backLeft,powerRight,powerLeft,targetPositionRight,
                targetPositionLeft,timeoutS,telemetry);

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // Detect whether center block is yellow
        boolean whiteMiddle = sensorHelper.isWhite(middleColor, telemetry);
        telemetry.addData("middleColor: ", whiteMiddle);
        telemetry.update();
        if (!whiteMiddle) {
            powerRight = 0.25;
            powerLeft = 0.25;
            targetPositionLeft = 4;
            targetPositionRight = 4;
            timeoutS = 5;
            motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight,
                    targetPositionLeft, timeoutS, telemetry);
            telemetry.addData("Travel Forward: ", "yes");
            telemetry.update();
            powerRight = -0.25;
            powerLeft = -0.25;
            targetPositionLeft = -4;
            targetPositionRight = -4;
            timeoutS = 5;
            motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight,
                    targetPositionLeft, timeoutS, telemetry);
        } else {
            final double EXTEND_OUT = 0.55;
            final double EXTEND_IN = 0.2;

            leftKnocker.setPosition(0.275);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            leftArm.setPosition(EXTEND_OUT);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            boolean whiteLeft = sensorHelper.isWhite(knockerColor,telemetry);
            telemetry.addData("leftColor: ", whiteLeft);
            telemetry.update();
            if (!whiteLeft) {

                //if left is yellow cube, left arm goes knocks off cube

                leftKnocker.setPosition(0);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                leftArm.setPosition(0.8);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                leftKnocker.setPosition(0.4);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                leftArm.setPosition(EXTEND_IN);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            } else {

                //else, the right side would have the cube, and it would knock it off with the right arm/knocker
                leftArm.setPosition(EXTEND_IN);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                rightKnocker.setPosition(0.6);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                rightArm.setPosition(0);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                rightKnocker.setPosition(0);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                rightArm.setPosition(0.6);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }

            //in case it doesn't go back up

            leftArm.setPosition(0);

            rightArm.setPosition(1);

            /*
            Testing the servo positions:

            leftKnocker.setPosition(0);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            rightKnocker.setPosition(0.6);

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            leftArm.setPosition(EXTEND_OUT);

            telemetry.addData("LeftArmPosition: ",EXTEND_OUT);
            telemetry.update();

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            rightArm.setPosition(0);

            telemetry.addData("RightArmPosition: ", 0.8);
            telemetry.update();

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            leftKnocker.setPosition(0.4);

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            leftKnocker.setPosition(0.6);

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            rightKnocker.setPosition(0);

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            leftArm.setPosition(EXTEND_IN);

            telemetry.addData("LeftArmPosition: ",EXTEND_IN);
            telemetry.update();

            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            */
        }

    }
}
