package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helper.MotorHelper;
import org.firstinspires.ftc.teamcode.helper.SensorHelper;


public class Sampling {
    //Commented on 01/21 - from 1000 to 500
    final long SLEEP_TIME_500 = 500;
    final long SLEEP_TIME_250 = 250;
    final double EXTEND_OUT = 0.55;
    final double EXTEND_IN = 0.2;
    double distToMinerals = 17.25;

    public void forwardWithColorSensor(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, MotorHelper motorHelper, SensorHelper sensorHelper,
                                       Telemetry telemetry, ColorSensor middleColor, DistanceSensor distanceSensor, Servo rightArm, Servo leftArm, Servo rightKnocker,
                                       Servo leftKnocker, ColorSensor knockerColor, ColorSensor rightKnockerColor) {

        //moving forward to minerals to begin detection
        //WITHOUT DISTANCE SENSOR - (fixed distance at 17.5")
        //UUUUNNNNNCCCCOOOOOOMMMMMMMEEEENNNNTTT!!!!!!

        double powerRight = 0.25;
        double powerLeft = 0.25;
        double targetPositionLeft = distToMinerals;
        double targetPositionRight = distToMinerals;
        double timeoutS = 5;
        motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight,
                targetPositionLeft, timeoutS, telemetry);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }





        //USING DISTANCE SENSOR - going forwards to minerals:
        //Going forward 15" then measuring distance
        //If measured distance is greater than 3"(tgtDistToMineral)
        //then set a new target position of the 'measured distance' minus 'tgtDistToMineral'
        //pass this difference into the moving with encoders method

        /*double tgtDistToMineral = 2.15;
        double powerRight = 0.25;
        double powerLeft = 0.25;
        double targetPositionLeft = 15;
        double targetPositionRight = 15;
        double timeoutS = 5;
        motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight,
                targetPositionLeft, timeoutS, telemetry);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        double distanceMeasured = (distanceSensor.getDistance(DistanceUnit.INCH));

        telemetry.addData("Distance from mineral: ", distanceMeasured);
        telemetry.update();
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(distanceMeasured > tgtDistToMineral) {
            double newTargetPosition = (distanceMeasured - tgtDistToMineral);
            telemetry.addData("Distance to reach mineral: ", newTargetPosition);
            telemetry.update();
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            powerRight = 0.25;
            powerLeft = 0.25;
            targetPositionLeft = newTargetPosition;
            targetPositionRight = newTargetPosition;
            timeoutS = 5;
            motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight,
                    targetPositionLeft, timeoutS, telemetry);

        } else {
            telemetry.addData("Distance from mineral: ", "sufficient!");
            telemetry.update();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //do nothing and move on to knocking the minerals
        } */




/*
lower right arm to color sense to right mineral
If it’s gold → knock it
bring right arm back into robot if not gold
put left arm down to color sense the left mineral
If it’s gold → knock it
bring left arm back into robot if not gold
move robot forward to knock middle mineral
move robot back to start heading to depot
*/

        // Detect whether center block is yellow
        /*boolean whiteMiddle = sensorHelper.isWhite(middleColor, telemetry);
        telemetry.addData("middleColor: ", whiteMiddle);
        telemetry.update();

        if (!whiteMiddle) {
            knockMiddleYellow(motorHelper, frontRight, frontLeft, backRight, backLeft, telemetry);

        } else */



        //bringing right arm down to sense the color of the right mineral
        //down is 0, up is 1 & out is 0, in is 1

        //rightKnocker.setPosition(0.48);
        //rightKnocker.setPosition(0.43);
        rightKnocker.setPosition(0.45);
        telemetry.addData("Right Knocker Position", rightKnocker.getPosition());
        telemetry.update();
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //right arm positions
        //rightArm.setPosition(0.725);
        rightArm.setPosition(0.675);
        telemetry.addData("Right Arm Position", rightArm.getPosition());
        telemetry.update();
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        boolean whiteRight = sensorHelper.isWhite(rightKnockerColor, telemetry);
        telemetry.addData("rightColor: ", whiteRight);
        telemetry.update();

        if (!whiteRight) {
            knockRightYellow(leftKnocker, leftArm, rightArm, rightKnocker);

        } else {
            //arm coming back in positions:
            rightArm.setPosition(0.8);
            try {
                Thread.sleep(SLEEP_TIME_500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            rightKnocker.setPosition(1);
            try {
                Thread.sleep(SLEEP_TIME_500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            //color sensor arm dropped down to determine if the left is yellow
            leftKnocker.setPosition(0.825);
            try {
                Thread.sleep(SLEEP_TIME_500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            leftArm.setPosition(0.6);
            try {
                Thread.sleep(SLEEP_TIME_500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            //color sense left mineral
            boolean whiteLeft = sensorHelper.isWhite(knockerColor, telemetry);
            telemetry.addData("leftColor: ", whiteLeft);
            telemetry.update();

            if (!whiteLeft) {
                //if left is yellow cube, left arm goes knocks off cube
                knockLeftYellow(leftKnocker, leftArm);

            } else {

                //moving left arm back in
                leftArm.setPosition(0.2);
                try {
                    Thread.sleep(SLEEP_TIME_500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                leftKnocker.setPosition(0);
                try {
                    Thread.sleep(SLEEP_TIME_500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //else, the middle position would have the cube, and the robot would move forward and back to knock it
                knockMiddleYellow(motorHelper, frontRight, frontLeft, backRight, backLeft, telemetry);
            }

            //in case it doesn't go back up, setting both arms back in:
            leftArm.setPosition(0);
            rightArm.setPosition(1);
        }
    }






    public void forwardWithTensor(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, MotorHelper motorHelper, SensorHelper sensorHelper,
                                  Telemetry telemetry, DistanceSensor distance, Servo rightArm, Servo leftArm, Servo rightKnocker,
                                  Servo leftKnocker, String yellowPosition) {
        //moving forward to minerals
        double powerRight = 0.25;
        double powerLeft = 0.25;
        double targetPositionLeft = distToMinerals ;
        double targetPositionRight = distToMinerals;
        double timeoutS = 5;
        motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight,
                targetPositionLeft, timeoutS, telemetry);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if (yellowPosition.equalsIgnoreCase("Center")) {
            knockMiddleYellow(motorHelper, frontRight, frontLeft, backRight, backLeft, telemetry);

        } else { if (yellowPosition.equalsIgnoreCase("Left")) {
                //if left is yellow cube, left arm goes knocks off cube
                knockLeftYellow(leftKnocker, leftArm);
            } else {
                //else, the right side would have the cube, and it would knock it off with the right arm/knocker
                knockRightYellow(leftKnocker, leftArm, rightArm, rightKnocker);
            }
            //in case it doesn't go back in, this does it:
            leftArm.setPosition(0);
            rightArm.setPosition(1);
        }
    }





    /*
    3 methods below are for Servo & Motor positions for knocking the gold cube when it's in the middle, left, or right:
    */

    //MIDDLE
    private void knockMiddleYellow(MotorHelper motorHelper, DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, Telemetry telemetry) {

        double powerRight = 0.25;
        double powerLeft = 0.25;
        double targetPositionLeft = 4;
        double targetPositionRight = 4;
        double timeoutS = 5;
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
    }

    //LEFT
    private void knockLeftYellow(Servo leftKnocker, Servo leftArm) {
        leftKnocker.setPosition(0.7);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //leftArm.setPosition(0.8);
        leftArm.setPosition(0.75);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        leftKnocker.setPosition(1);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        leftArm.setPosition(0.2);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        leftKnocker.setPosition(0);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    //RIGHT
    private void knockRightYellow(Servo leftKnocker, Servo leftArm, Servo rightArm, Servo rightKnocker) {

        leftArm.setPosition(0.2);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        leftKnocker.setPosition(0);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        rightKnocker.setPosition(0.6);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        rightArm.setPosition(0);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        rightKnocker.setPosition(0.3);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //arm coming back in positions:

        rightArm.setPosition(0.8);
        //rightArm.setPosition(0.6);
        //rightArm.setPosition(0.75);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        rightKnocker.setPosition(1);
        try {
            Thread.sleep(SLEEP_TIME_500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

}
