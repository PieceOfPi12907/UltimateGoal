package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.opmode.AutoAll;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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

public class WebcamThreadTester extends LinearOpMode {

    DcMotor movementMotor1;

    BNO055IMU imu;

    WebcamName webcam;
    OpenGLMatrix lastLocation;
    Thread webcamInit;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    VuforiaLocalizer.Parameters parametersWebcam = null;
    final float mmPerInch = 25.4f;
    double y_value = 0;
    VuforiaTrackables targetsSkyStone;

    NavigationHelper navigationHelper;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if(opModeIsActive()) {
            navigationHelper.navigate(11, Constants12907.Direction.RIGHT, 0, 0.5, movementMotor1, movementMotor1, movementMotor1, movementMotor1, imu, telemetry);
            webcamInit.interrupt();
            Constants12907.SkystonePosition skystonePosition = detectSkystoneWithWebcam(lastLocation);

            if (skystonePosition.equals(Constants12907.SkystonePosition.LEFT)) {
                telemetry.addLine("LEFT");

            }
            if (skystonePosition.equals(Constants12907.SkystonePosition.RIGHT)) {
                telemetry.addLine("RIGHT");

            }
            if (skystonePosition.equals(Constants12907.SkystonePosition.CENTER)) {
                telemetry.addLine("CENTER");

            }
            telemetry.update();
        }
    }

    public void initialize(){
        movementMotor1 = hardwareMap.get(DcMotor.class, "movementMotor1");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        webcam = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parametersWebcam = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        webcamInit = new WebcamThreadTester.WebcamThread();
        webcamInit.run();

    }

    public class WebcamThread extends Thread {

        public void webcamThread() {
            this.setName("Webcam Thread");
        }

        @Override
        public void run(){
            webcam = hardwareMap.get(WebcamName.class, "webcam");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parametersWebcam = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
            final boolean PHONE_IS_PORTRAIT = false;
            final String VUFORIA_KEY =
                    "AU29nsP/////AAABmbmvuOwXSkEvjnmA/GE4vvtYcQ++rPSuF0c4fwBMDyTKVMiy0tgOzM+wgd2h5lhtywdhWMQRV+FPhX1SKEZ2LYwl1jKMN4JaKaWikc8DEfoXeFYf5cRAzlQa8CGQ2IFKgYm9Dq5tk8pdrD9WYqb4OFOUW6QkqhiOR1UCTQrAxgqCX0duHNRNK3ksVOyfDszUPL9r5nbIuaISyP5/iN7hWTbRk9damSem6xmKX4yex2YBroO0Ly7BX+JOiuu6x7c059WReN6DU1hrBDwUhIXxKjdV9OOTFL9uw1xedulivMI4G5LbjlQks09aSm/BbfUpCygx8oFo6NLikKP7V5RGUZBfOBwIP/cZDEb52gUZiBcp";
// Constant for Stone Target
            final float stoneZ = 2.00f * mmPerInch;
            //OpenGLMatrix lastLocation = null;
            VuforiaLocalizer vuforia = null;
            float phoneXRotate = 0;
            float phoneYRotate = 0;
            float phoneZRotate = 0;
            boolean targetVisible = false;
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
             * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
             */
            parametersWebcam.vuforiaLicenseKey = VUFORIA_KEY;
            parametersWebcam.cameraName = webcam;

            vuforia = ClassFactory.getInstance().createVuforia(parametersWebcam);

// Load the data sets for the trackable objects. These particular data
// sets are stored in the 'assets' part of our application.
            targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");


            VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");



            allTrackables = new ArrayList<VuforiaTrackable>();
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
                phoneXRotate = 90;
            }

// Next, translate the camera lens to where it is on the robot.
// In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
            final float CAMERA_FORWARD_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
            final float CAMERA_VERTICAL_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));



            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parametersWebcam.cameraDirection);
            }

            VectorF translation = null;
            targetsSkyStone.activate();

            while (!isInterrupted()) {
                targetsSkyStone.activate();
                targetVisible = false;


                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;
                        targetsSkyStone.deactivate();

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
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
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    y_value = translation.get(1) / mmPerInch;

                } else {
                    telemetry.addData("Visibsle Target", "none");
                }
                telemetry.update();

            }

        }
    }

    public Constants12907.SkystonePosition detectSkystoneWithWebcam(OpenGLMatrix lastLocation){//Telemetry pTelemetry, WebcamName pWebcam, VuforiaLocalizer.Parameters pParameters, ElapsedTime detectingTime, boolean blockFound, boolean targetVisible, OpenGLMatrix lastLocation, VectorF translation, float mmPerInch, double y_value, List<VuforiaTrackable> allTrackables, VuforiaTrackables targetsSkyStone ) {
        ElapsedTime detectingTime = new ElapsedTime();
        detectingTime.reset();
        boolean blockFound = false;
        boolean targetVisible = false;
        VectorF translation = null;
        while (detectingTime.seconds() <= 1 && !blockFound) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
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
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                y_value = translation.get(1) / mmPerInch;
                blockFound = true;

            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();

        }
        Constants12907.SkystonePosition blockPosition = Constants12907.SkystonePosition.RIGHT;

        if (targetVisible == false) {
            blockPosition = Constants12907.SkystonePosition.LEFT;
            telemetry.addLine("********!!!!!!POS (LEFT)");
        } else if (y_value < 0) {
            blockPosition = Constants12907.SkystonePosition.CENTER;
            if (translation != null) {
                telemetry.addData("********!!!!!!POS (CENTER): ", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            }
        } else if (y_value > 0) {
            blockPosition = Constants12907.SkystonePosition.RIGHT;

            if (translation != null) {
                telemetry.addData("********!!!!!!POS (RIGHT): ", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            }
        }

        telemetry.update();

        return blockPosition;

    }

}
