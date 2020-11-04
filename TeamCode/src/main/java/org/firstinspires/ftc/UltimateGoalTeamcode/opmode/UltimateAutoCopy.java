package org.firstinspires.ftc.UltimateGoalTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.UltimateGoalTeamcode.helper.DetectionHelper;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "ULTIMATE AUTO COPY", group = "autonomous")
public class UltimateAutoCopy extends LinearOpMode {

    Boolean isBlue = false;
    Boolean isWall = false;
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    OpenCvCamera webcam;
    DetectionHelper pipeline;

    public void initialize() {

        isBlue = false;
        isWall = false;

        //Initializing the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new DetectionHelper();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
        telemetry.update();

        DetectionHelper.RingPosition position = null;

        while(!opModeIsActive()) {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.getPosition());
            position = pipeline.getPosition();
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            //int num = detectionLoop();
            telemetry.addData("Number of Rings", position);
            telemetry.update();
            sleep(1000);
        }
    }

    public int detectionLoop(){
        double begin = System.currentTimeMillis()/1000;
        double end = System.currentTimeMillis()/1000;
        DetectionHelper.RingPosition position = null;
        while(end-begin<1.5) {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.getPosition());
            position = pipeline.getPosition();
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            end = System.currentTimeMillis()/1000;
        }
        if(position.equals(DetectionHelper.RingPosition.FOUR)){
            return 4;
        }
        else if(position.equals(DetectionHelper.RingPosition.ONE)){
            return 1;
        }
        else if(position.equals(DetectionHelper.RingPosition.NONE)){
            return 0;
        }
        return 3;
    }


}
