package org.firstinspires.ftc.SkystoneTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
public class TensorTesterClass extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    ElapsedTime detection = new ElapsedTime();
    String position = "RIGHT";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AU29nsP/////AAABmbmvuOwXSkEvjnmA/GE4vvtYcQ++rPSuF0c4fwBMDyTKVMiy0tgOzM+wgd2h5lhtywdhWMQRV+FPhX1SKEZ2LYwl1jKMN4JaKaWikc8DEfoXeFYf5cRAzlQa8CGQ2IFKgYm9Dq5tk8pdrD9WYqb4OFOUW6QkqhiOR1UCTQrAxgqCX0duHNRNK3ksVOyfDszUPL9r5nbIuaISyP5/iN7hWTbRk9damSem6xmKX4yex2YBroO0Ly7BX+JOiuu6x7c059WReN6DU1hrBDwUhIXxKjdV9OOTFL9uw1xedulivMI4G5LbjlQks09aSm/BbfUpCygx8oFo6NLikKP7V5RGUZBfOBwIP/cZDEb52gUZiBcp";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    public Constants12907.SkystonePosition detect(){
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        detection.reset();
        while (detection.seconds()<=1) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if(recognition.getLabel().equals("Skystone")) {
                              /*telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                              telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                      recognition.getLeft(), recognition.getTop());
                              telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                      recognition.getRight(), recognition.getBottom());

                               */
                            if(recognition.getLeft()<300){
                                telemetry.addData("Skystone position","LEFT");
                                position = "LEFT";
                            }
                            if(recognition.getRight()>500){
                                telemetry.addData("Skystone position","CENTER");
                                position = "CENTER";
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        if(position.equals("LEFT")){
            return Constants12907.SkystonePosition.LEFT;
        }
        else if(position.equals("CENTER")){
            return Constants12907.SkystonePosition.CENTER;
        }
        else{
            return Constants12907.SkystonePosition.RIGHT;
        }

    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            detection.reset();
            while (detection.seconds()<=1) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getLabel().equals("Skystone")) {
                              /*telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                              telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                      recognition.getLeft(), recognition.getTop());
                              telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                      recognition.getRight(), recognition.getBottom());

                               */
                                if(recognition.getLeft()<300){
                                    telemetry.addData("Skystone position","LEFT");
                                    position = "LEFT";
                                }
                                if(recognition.getRight()>500){
                                    telemetry.addData("Skystone position","CENTER");
                                    position = "CENTER";
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
            telemetry.addData("Final position found", position);
            telemetry.update();
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}