package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.helper.SensorHelper;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class SkystoneDetection {


    final double PIVOT_LOWERED = 0.9;

    final double PIVOT_RAISED = 0.25;

    final double CLAMP_OPENED = 0.5;
    final double CLAMP_CLOSED = 0.8;

    int negative;
    double correction;
    double shift;
    double imu_correct;

    NavigationHelper navigater = new NavigationHelper();
    SensorHelper sensorHelper = new SensorHelper();

    public void moveToSkystoneOuter(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeft, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, Boolean isBlue, Boolean isPos2) {
        if (isBlue==false) {
            negative = -1;
            correction = 2;
        } else {
            negative = 1;
            correction = 0;
        }
        if(isPos2==true) {
            pNavigate.navigate(22.75*negative, Constants12907.Direction.STRAIGHT, 0, 0.5*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        }

        //Strafing to the Skystone - sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
        pNavigate.navigate(25, Constants12907.Direction.RIGHT, 0, 0.35, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        double currentDistance = quarryDistance.getDistance(DistanceUnit.INCH);
        double targetDistance = 3;

        pTelemetry.addData("DISTANCE TO QUARRY: ", currentDistance);
        pTelemetry.update();

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        pNavigate.navigate(currentDistance - targetDistance, Constants12907.Direction.RIGHT, 0, 0.2, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        Constants12907.SkystonePosition blackBlockPosition = sensorHelper.getBlackBlock(colorLeft,colorRight, pTelemetry);

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //RETURNS POSITION 1, 2, OR 3 FROM 'getBlackBlock' METHOD IN SensorHelper
        pTelemetry.addData("SKYSTONE POSITION from getBlackBlock",blackBlockPosition.toString());

        //robot moves to pick up skystone based on what position it is in:
        if (blackBlockPosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pNavigate.navigate((15-correction)*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-(13-correction)*negative, Constants12907.Direction.STRAIGHT, 0, -0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else if (blackBlockPosition.equals(Constants12907.SkystonePosition.CENTER) ) {
            pNavigate.navigate((8-correction)*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-(6-correction)*negative, Constants12907.Direction.STRAIGHT, 0, -0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else {
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate((2-correction)*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        }

        pNavigate.navigate(27, Constants12907.Direction.LEFT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
        pNavigate.navigate(3, Constants12907.Direction.RIGHT,0,0.5,pBackLeft,pBackRight,pFrontRight,pFrontLeft,pImu,pTelemetry);
    }







    public void moveToSkystoneInner(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, ColorSensor colorRight, ColorSensor colorLeft, DistanceSensor distanceRight, DistanceSensor distanceLeft, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, Boolean isBlue, Boolean isPos2) {

        if (isBlue==false) {
            negative = -1;
            correction = 2;
            shift = 5;
        } else {
            negative = 1;
            correction = 0;
            shift = 0;
        }
        if(isPos2==true) {
            pNavigate.navigate(22.75*negative, Constants12907.Direction.STRAIGHT, 0, 0.5*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        }

        //Strafing to the Skystone - sped up complete: 0.25 --> 0.5, 0.2 --> 0.4, 0.4 --> 0.7
        pNavigate.navigate(25, Constants12907.Direction.RIGHT, 0, 0.35, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        double currentDistance = quarryDistance.getDistance(DistanceUnit.INCH);
        double targetDistance = 2.25;

        pTelemetry.addData("DISTANCE TO QUARRY: ", currentDistance);
        pTelemetry.update();

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        pNavigate.navigate(currentDistance - targetDistance, Constants12907.Direction.RIGHT, 0, 0.2, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        Constants12907.SkystonePosition blackBlockPosition = sensorHelper.getBlackBlock(colorLeft,colorRight, pTelemetry);

        //telemetry for reading the color sensor values
        pTelemetry.addData("Red Right:  ", colorRight.red());
        pTelemetry.addData("Green Right: ", colorRight.green());
        pTelemetry.addData("Red Left:  ", colorLeft.red());
        pTelemetry.addData("Green Left: ", colorLeft.green());
        pTelemetry.update();

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //RETURNS POSITION 1, 2, OR 3 FROM 'getBlackBlock' METHOD IN SensorHelper
        pTelemetry.addData("SKYSTONE POSITION from getBlackBlock",blackBlockPosition.toString());

        //robot moves to pick up skystone based on what position it is in:
        if (blackBlockPosition.equals(Constants12907.SkystonePosition.LEFT)) {

            pNavigate.navigate((15-correction)*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-(13-correction)*negative, Constants12907.Direction.STRAIGHT, 0, -0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else if (blackBlockPosition.equals(Constants12907.SkystonePosition.CENTER) ) {

            pNavigate.navigate((8-correction)*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-(6-correction)*negative, Constants12907.Direction.STRAIGHT, 0, -0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else {

            pNavigate.navigate((shift-correction)*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate((2-(correction+shift))*negative, Constants12907.Direction.STRAIGHT, 0, 0.25*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        }

        pNavigate.navigate(3, Constants12907.Direction.LEFT, 0, 0.2, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
    }







    public void intakeSkystone(Servo blockClamper, Servo pivotGrabber) {
        //Intake skystone  Code
        blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_LOWERED);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_CLOSED);
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        pivotGrabber.setPosition(PIVOT_RAISED);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }





   public Constants12907.SkystonePosition detectSkystoneWithWebcam(Telemetry pTelemetry, WebcamName pWebcam, VuforiaLocalizer.Parameters pParameters){
        ElapsedTime detectingTime = new ElapsedTime();
        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
        final boolean PHONE_IS_PORTRAIT = false  ;
        final String VUFORIA_KEY =
                "AU29nsP/////AAABmbmvuOwXSkEvjnmA/GE4vvtYcQ++rPSuF0c4fwBMDyTKVMiy0tgOzM+wgd2h5lhtywdhWMQRV+FPhX1SKEZ2LYwl1jKMN4JaKaWikc8DEfoXeFYf5cRAzlQa8CGQ2IFKgYm9Dq5tk8pdrD9WYqb4OFOUW6QkqhiOR1UCTQrAxgqCX0duHNRNK3ksVOyfDszUPL9r5nbIuaISyP5/iN7hWTbRk9damSem6xmKX4yex2YBroO0Ly7BX+JOiuu6x7c059WReN6DU1hrBDwUhIXxKjdV9OOTFL9uw1xedulivMI4G5LbjlQks09aSm/BbfUpCygx8oFo6NLikKP7V5RGUZBfOBwIP/cZDEb52gUZiBcp";
        final float mmPerInch        = 25.4f;
        // Constant for Stone Target
        final float stoneZ = 2.00f * mmPerInch;
        OpenGLMatrix lastLocation = null;
        VuforiaLocalizer vuforia = null;
        float phoneXRotate    = 0;
        float phoneYRotate    = 0;
        float phoneZRotate    = 0;
        boolean targetVisible = false;
        double y_value=0;
        boolean blockFound = false;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        pParameters.vuforiaLicenseKey = VUFORIA_KEY;
        pParameters.cameraName = pWebcam;

        vuforia = ClassFactory.getInstance().createVuforia(pParameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.add(targetsSkyStone.get(0));

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 8.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, pParameters.cameraDirection);
        }

        targetsSkyStone.activate();
        detectingTime.reset();

       VectorF translation = null;

        while(detectingTime.seconds()<=5 && !blockFound){
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    pTelemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }//for

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                translation = lastLocation.getTranslation();
                pTelemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                y_value = translation.get(1)/mmPerInch;
                blockFound = true;

            }
            else {
                pTelemetry.addData("Visible Target", "none");
            }
            pTelemetry.update();

        }
        Constants12907.SkystonePosition blockPosition = Constants12907.SkystonePosition.RIGHT;

        if(targetVisible==false){
            blockPosition = Constants12907.SkystonePosition.LEFT;
            pTelemetry.addLine("********!!!!!!POS (LEFT)");
        }
        else if(y_value<0){
            blockPosition = Constants12907.SkystonePosition.CENTER;
            if(translation != null){
                pTelemetry.addData("********!!!!!!POS (CENTER): ", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            }
        }
        else if (y_value>0){
            blockPosition = Constants12907.SkystonePosition.RIGHT;

            if(translation != null){
                pTelemetry.addData("********!!!!!!POS (RIGHT): ", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            }
        }

        pTelemetry.update();

        return blockPosition;

    }





        //METHOD BEING CURRENTLY USED:
    public void moveToSkystoneOne(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, SkystoneDelivery skystoneDelivery, Boolean isBlue){

        if(isBlue == true){
            negative = 1;
        } else {
            negative = -1;
        }

        pivotGrabber.setPosition(0.65);

        double currentDistance = quarryDistance.getDistance(DistanceUnit.INCH);
        double targetDistance = 1.5;

        pTelemetry.addData("DISTANCE TO QUARRY: ", currentDistance);

        pTelemetry.update();

        if(currentDistance > 18){
            currentDistance = 17;
        }

        double toGoDistance = currentDistance - targetDistance;

        pTelemetry.addData("STRAFE DISTANCE ", toGoDistance);

        pNavigate.navigate(toGoDistance, Constants12907.Direction.RIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        double newDistance = quarryDistance.getDistance(DistanceUnit.INCH);
        pTelemetry.addData("NEW DISTANCE TO QUARRY", newDistance);

        pTelemetry.update();

        if(isBlue == true){
            goToSkystoneOneBlue(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, skystoneDelivery, isBlue);
        } else {
            goToSkystoneOneRed(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, skystoneDelivery, isBlue);
        }

        pNavigate.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        pNavigate.navigate(44*negative, Constants12907.Direction.STRAIGHT, 0, 0.60*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        skystoneDelivery.extakeSkystone(blockClamper, pivotGrabber);
    }




    private void goToSkystoneOneRed(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, SkystoneDelivery skystoneDelivery, Boolean isBlue){
        if (pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("position --> LEFT");

            pNavigate.navigate(8, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-18, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else if (pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER) ) {
            pTelemetry.addLine("position --> CENTER");

            pNavigate.navigate(-2, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-8, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else {

            pTelemetry.addLine("position --> RIGHT");

            pNavigate.navigate(-8, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-2, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        }
    }


    private void goToSkystoneOneBlue(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, SkystoneDelivery skystoneDelivery, Boolean isBlue){
        if (pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("position --> LEFT");

            pNavigate.navigate((12), Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate((0), Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else if (pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER) ) {
            pTelemetry.addLine("position --> CENTER");

            pNavigate.navigate((6), Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate((6), Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
//8
        } else {

            pTelemetry.addLine("position --> RIGHT");

            pNavigate.navigate(-(2), Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            imu_correct = pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate((14), Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
//16
        }
    }










    public void moveToSkystoneTwo(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, SkystoneDelivery skystoneDelivery, boolean isBlue){

        if(isBlue == true){
            negative = 1;
            shift = 0;

            pNavigate.navigate(-70*negative, Constants12907.Direction.STRAIGHT, 0, -0.65*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else {
            negative = -1;
            pNavigate.navigate(-74*negative, Constants12907.Direction.STRAIGHT, 0, -0.65*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        }

        pivotGrabber.setPosition(0.65);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        blockClamper.setPosition(CLAMP_OPENED);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        double currentDistance = quarryDistance.getDistance(DistanceUnit.INCH);
        double targetDistance = 1.5;

        pTelemetry.addData("DISTANCE TO QUARRY: ", currentDistance);

        pTelemetry.update();

        if(currentDistance > 18){
            currentDistance = 17;
        }

        double toGoDistance = currentDistance - targetDistance;

        pTelemetry.addData("STRAFE DISTANCE ", toGoDistance);

        if(isBlue != true){
            rightStrafeWithoutCorrection(toGoDistance, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else {
            pNavigate.navigate(toGoDistance, Constants12907.Direction.RIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        }

        double newDistance = quarryDistance.getDistance(DistanceUnit.INCH);

        pTelemetry.addData("NEW DISTANCE TO QUARRY", newDistance);

        pTelemetry.update();

       if(isBlue == true){
            goToSkystoneTwoBlue(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, skystoneDelivery, isBlue);
        } else {
            goToSkystoneTwoRed(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, skystoneDelivery, isBlue);
        }

        pNavigate.navigate(2.5, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        pNavigate.navigate(70*negative, Constants12907.Direction.STRAIGHT, 0, 0.75*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
//68
        skystoneDelivery.extakeSkystone(blockClamper, pivotGrabber);

        pNavigate.navigate(-22*negative, Constants12907.Direction.STRAIGHT, 0, -0.6*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
//-20
    }

    private void goToSkystoneTwoBlue(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, SkystoneDelivery skystoneDelivery, Boolean isBlue){
        if (pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("2nd position --> LEFT");
            pNavigate.navigate((0), Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate((0), Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else if (pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER) ) {
            pTelemetry.addLine("2nd position --> CENTER");
            pNavigate.navigate(-(8), Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate((8), Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        } else {
            pTelemetry.addLine("2nd position --> RIGHT");
            pNavigate.navigate((-16), Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate((16), Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        }
    }

    private void goToSkystoneTwoRed(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, SkystoneDelivery skystoneDelivery, Boolean isBlue){
        if (pSkystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
            pTelemetry.addLine("2nd position --> LEFT");
            pNavigate.navigate(8, Constants12907.Direction.STRAIGHT, 0, 0.6, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-10, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else if (pSkystonePosition.equals(Constants12907.SkystonePosition.CENTER) ) {
            pTelemetry.addLine("2nd position --> CENTER");
            pNavigate.navigate(4, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            intakeSkystone(blockClamper, pivotGrabber);
            pNavigate.navigate(-6, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        } else {
            pTelemetry.addLine("2nd position --> RIGHT");
            pNavigate.navigate(-2, Constants12907.Direction.STRAIGHT, 0, -0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

            double imuCorrection = (pImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)-imu_correct;
            pTelemetry.addData("IMU CORRECTION: ", imuCorrection);
            pTelemetry.update();
            navigater.turnWithEncoders(pFrontRight, pFrontLeft,pBackRight, pBackLeft, -imuCorrection, 0.25, pImu, pTelemetry);

            intakeSkystone(blockClamper, pivotGrabber);

            pNavigate.navigate(0, Constants12907.Direction.STRAIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        }
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



    public void moveToSkystoneOneWithREPO(DcMotor pBackLeft, DcMotor pBackRight, DcMotor pFrontRight, DcMotor pFrontLeft, NavigationHelper pNavigate, BNO055IMU pImu, Telemetry pTelemetry, Constants12907.SkystonePosition pSkystonePosition, DistanceSensor quarryDistance, Servo pivotGrabber, Servo blockClamper, SkystoneDelivery skystoneDelivery, Boolean isBlue){

        if(isBlue == true){
            negative = 1;
        } else {
            negative = -1;
        }

        pivotGrabber.setPosition(0.65);

        double currentDistance = quarryDistance.getDistance(DistanceUnit.INCH);
        double targetDistance = 1.5;

        pTelemetry.addData("DISTANCE TO QUARRY: ", currentDistance);

        double toGoDistance = currentDistance - targetDistance;

        pTelemetry.addData("STRAFE DISTANCE ", toGoDistance);

        pNavigate.navigate(toGoDistance, Constants12907.Direction.RIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        double newDistance = quarryDistance.getDistance(DistanceUnit.INCH);
        pTelemetry.addData("NEW DISTANCE TO QUARRY", newDistance);

        pTelemetry.update();

        if(isBlue == true){
            goToSkystoneOneBlue(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, skystoneDelivery, isBlue);
        } else {
            goToSkystoneOneRed(pBackLeft, pBackRight, pFrontRight, pFrontLeft, pNavigate,  pImu, pTelemetry, pSkystonePosition, quarryDistance, pivotGrabber, blockClamper, skystoneDelivery, isBlue);
        }

        pNavigate.navigate(5, Constants12907.Direction.LEFT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        pNavigate.navigate(64*negative, Constants12907.Direction.STRAIGHT, 0, 0.60*negative, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        //pNavigate.navigate(10, Constants12907.Direction.RIGHT, 0, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);
        rightStrafeWithoutCorrection(10, 0.25, pBackLeft, pBackRight, pFrontRight, pFrontLeft, pImu, pTelemetry);

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        skystoneDelivery.extakeSkystone(blockClamper, pivotGrabber);

    }


}