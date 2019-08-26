/*package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helper.MotorHelper;

@Autonomous(name = "AutonomousTest_Encoders", group = "autonomous")
public class Autonomous12907 extends LinearOpMode {
    //Naming the motors
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    MotorHelper motorHelper;

    public void initialize() {
        //Configuration of the motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        //setting the directions of the motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        motorHelper = new MotorHelper();
        double powerRight = 0.25;
        double powerLeft = 0.25;
        double targetPositionLeft = 12;
        double targetPositionRight = 12;
        double timeoutS = 5;
        if (opModeIsActive()) {
            motorHelper.movingWithEncoders(frontRight, frontLeft, backRight, backLeft,
                    powerRight, powerLeft,
                    targetPositionRight, targetPositionLeft,
                    timeoutS, telemetry);
        }
    }
}
*/

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.helper.MotorHelper;
import org.firstinspires.ftc.teamcode.helper.SensorHelper;
import org.firstinspires.ftc.teamcode.utilities.Landing;
import org.firstinspires.ftc.teamcode.utilities.Marker;
import org.firstinspires.ftc.teamcode.utilities.Parking;
import org.firstinspires.ftc.teamcode.utilities.Sampling;

import java.util.List;

//Adding Source Code to GitHub

@Autonomous(name = "Autonomous 2019", group = "autonomous")
public class Autonomous12907 extends LinearOpMode {


    final long SLEEP_TIME_250 = 250;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYJ3N6D/////AAABmZ2Icf/PZ0ajiJKTK1ljgutPstOUxpIqfSB7xP/OphqNsQryq3m1pawYHoXK6D06FiGPO7zMgXFhVGSY82cswYVPq9JTE6hivLI9pxYpTxx+sMTae4URQ7xoSNdrHOQMsKjcm4NTuEJCXgDEAlBKwh4f1Hk/64IyMaM7Lyyr6cGj2l+8B9seDzroPaXPdgqAnMvJDpEB+YAS/rT93NygTlk+2zBddTZeNIaT4rHuHjebkm6xtovRUJykMBCRf2E+JJVMWiexqp7F5OMJaK+u6ceDMjbG/2yrojiMVTWAwdDRSre1RnkYotxxby1jxxSh3NFFmaRe/4vPi2r1xTs1nX4R61aJIaqpVXfLk1yX3drM";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    //Naming the motors
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    Servo latch;
    //Servo markerDropper;
    Servo sweeperDump;
    Servo sweeperDumpRight;
    Servo sweeperDumpLeft;
    Servo rightArm;
    Servo leftArm;
    Servo rightKnocker;
    Servo leftKnocker;
    DcMotor liftActuator;
    DcMotor sweeper;
    BNO055IMU imu;
    DistanceSensor distanceSensor;
    Orientation lastAngles;
    ColorSensor knockerColor;
    ColorSensor markerColor;
    ColorSensor middleColor;
    ColorSensor rightKnockerColor;
    DistanceSensor distanceColor;
    boolean turnRight;

    //Initializes motors from the hardware map

    public void initialize() {
        //Configuration of the motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");
        liftActuator = hardwareMap.get(DcMotor.class, "lift");
        latch = hardwareMap.get(Servo.class, "latch");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightKnocker = hardwareMap.get(Servo.class, "rightKnocker");
        leftKnocker = hardwareMap.get(Servo.class, "leftKnocker");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        //markerDropper = hardwareMap.get(Servo.class, "markerDropper");
        sweeperDump = hardwareMap.get(Servo.class, "sweeperDump");
        sweeperDumpRight = hardwareMap.get(Servo.class, "sweeperDumpRight");
        sweeperDumpLeft = hardwareMap.get(Servo.class, "sweeperDumpLeft");
        markerColor = hardwareMap.get(ColorSensor.class, "markerColor");
        middleColor = hardwareMap.get(ColorSensor.class, "middleColor");
        knockerColor = hardwareMap.get(ColorSensor.class, "knockerColor");
        distanceColor = hardwareMap.get(DistanceSensor.class, "middleColor");
        rightKnockerColor = hardwareMap.get(ColorSensor.class, "rightKnockerColor");

        //Setting the direction of the motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftActuator.setDirection(DcMotorSimple.Direction.FORWARD);

        //color sensing the color of the black/white chip to see which direction to turn
        // depending on if the robot is on crater or depot side
        SensorHelper sensorHelper = new SensorHelper();
        turnRight = sensorHelper.isWhite(markerColor, telemetry);
        telemetry.addData("turnRight: ",turnRight);
        telemetry.update();
        try {
            Thread.sleep(SLEEP_TIME_250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //Initializing the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }


    @Override
    public void runOpMode() throws InterruptedException {

        try {
            initialize();

            Landing landing = new Landing();
            Sampling sampling = new Sampling();
            Marker marker = new Marker();
            MotorHelper motorHelper = new MotorHelper();
            SensorHelper sensorHelper = new SensorHelper();

            //TensorCode:
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.

            initVuforia();
            boolean canUseTensor;
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
                canUseTensor = true;
                telemetry.addData("Yes", "Tensor Flow can be used");
                //added rest to view telemetry
                try {
                    Thread.sleep(SLEEP_TIME_250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                canUseTensor = false;
            }


            waitForStart();

            if (opModeIsActive()) {
                //calling tensor (same code as ConceptTensorFlowObjectDetection)
                String yellowPosition = "Unknown";
                if (canUseTensor & turnRight) {
                    yellowPosition = detectYellowPosition();
                }


                //initializing sweeper box inwards for start of autonomous
                //initializing the right and left servos for the sweeper box
                    //sweeperDumpRight.setPosition(0.7);
                    //sweeperDumpLeft.setPosition(0.17);
                sweeperDumpRight.setPosition(0.9);
                sweeperDumpLeft.setPosition(0);
                try {
                    Thread.sleep(SLEEP_TIME_250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                /*sweeperDump.setPosition(1);
                try {
                    Thread.sleep(SLEEP_TIME_250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }*/


                //landing then unlatching - WORKING
                landing.drop(liftActuator, latch, motorHelper, telemetry);

                //moving forward for Sampling - WORKING
                //this disables tensorFlow for the crater side - as it will only use color sensing
                if (yellowPosition.equalsIgnoreCase("Unknown") || !turnRight) {
                    sampling.forwardWithColorSensor(frontRight, frontLeft, backRight, backLeft, motorHelper, sensorHelper, telemetry, middleColor, distanceSensor, rightArm, leftArm, rightKnocker, leftKnocker, knockerColor, rightKnockerColor);

                } else {
                    sampling.forwardWithTensor(frontRight, frontLeft, backRight, backLeft, motorHelper, sensorHelper, telemetry, distanceSensor, rightArm, leftArm, rightKnocker, leftKnocker, yellowPosition);
                }

                //moving to depot - WORKING
                marker.dropMarkerToDepot(frontRight, frontLeft, backRight, backLeft, motorHelper, telemetry, imu, sweeperDump, sweeperDumpRight,sweeperDumpLeft, sweeper,  turnRight);
            }
        } catch (Exception bad){
            telemetry.addData("EXCEPTION:", bad.toString());
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


















    /*
    Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /*
    Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }



    /**
    Detecting the gold mineral using modified tensor flow
     A picture of the right two minerals are taken
     The gold mineral and silver mineral are initially set to a value of -1
     The goldmineralX is either -1 if its a left most mineral or a positive number if its center or right
     & the silverMineralX will be a positive number as there will always be at least one silver mineral (once picture taken)
        First off, the silver mineral value is checked to see that it is not -1 (the silver mineral must be assigned a value- as there are two of them)
        If the gold mineral has a value of -1, it means it has not been assigned a 1 or 2 value (center of right position); so it is in the left position
        If the gold mineral's value is less than the silver mineral's value; it means that the gold mineral has a value of 1 & is in the center position
        By process of elimination, the gold mineral would be in the right position if not in the left or center position
     */

    public String detectYellowPosition() {
        ElapsedTime runTime = new ElapsedTime();

        String yellowPosition = "Unknown";

        //Activate Tensor Flow Object Detection.
        if (tfod != null) {
            tfod.activate();
        }
        runTime.reset();
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        while (true) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    //added sleep to see telemetry
                    //check to make sure we have only two objects
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        //int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("Recognition Height & Width:", recognition.getImageHeight() & recognition.getImageWidth());
                            telemetry.update();

                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            }
                        }
                        if (silverMineral1X != -1 ) {
                            if (goldMineralX ==-1) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                yellowPosition = "Left";

                            } else if (goldMineralX < silverMineral1X ) {
                                telemetry.addData("Gold Mineral Position", "Center");
                                yellowPosition = "Center";

                            } else {
                                telemetry.addData("Gold Mineral Position", "Right");
                                yellowPosition = "Right";
                            }
                        }

                    }

                    telemetry.update();


                    //Breaking tensor flow if objects detected is 3 or if it times out (after 2 seconds)
                    if (updatedRecognitions.size() == 2 || runTime.milliseconds() > 3000) {
                       /* telemetry.addData("Position: ", yellowPosition);
                        telemetry.addData("Objects Detected: ", updatedRecognitions.size());
                        telemetry.addData("Elapsed Time: ", runTime.milliseconds());
                        telemetry.update();
                        try {
                            Thread.sleep(2000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }*/
                        break;

                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        return yellowPosition;
    }


}



