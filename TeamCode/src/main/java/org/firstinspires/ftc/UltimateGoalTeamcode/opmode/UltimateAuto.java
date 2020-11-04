package org.firstinspires.ftc.UltimateGoalTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.Constants2020;
import org.firstinspires.ftc.UltimateGoalTeamcode.utilities.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.HashMap;

@Autonomous(name = "ULTIMATE AUTO", group = "autonomous")
public class UltimateAuto extends LinearOpMode {

    Boolean isBlue = false;
    Boolean isWall = false;
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    OpenCvCamera webcam;
    WobbleGoal wobbleGoal;

    HashMap<String, Object> variableMap = new HashMap<String, Object>();

    public void initialize() {

        isBlue = false;
        isWall = false;

        telemetry.addLine("reached initialization");
        telemetry.update();

        while(true) {
            //x on gamepad 1 sets BlUE
            if (gamepad1.x) {
                isBlue = true;
                telemetry.addLine("BLUE");
                telemetry.update();
            }

            //b on gamepad 1 sets RED
            if (gamepad1.b) {
                isBlue = false;
                telemetry.addLine("RED");
                telemetry.update();
            }

            //left dpad on gamepad 1 sets OUTER
            if (gamepad1.dpad_left) {
                isWall = true;
                telemetry.addLine("OUTER");
                telemetry.update();
            }

            //right dpad on gamepad 1 sets INNER
            if (gamepad1.dpad_right) {
                isWall = false;
                telemetry.addLine("INNER");
                telemetry.update();
            }

            //y on gamepad 1 CONFIRMS decisions
            if (gamepad1.a){
                telemetry.addData("COLOR: ", (isBlue == true)? "blue" : "red");
                telemetry.addData("ROUTE: ", (isWall == true)? "wall" : "not wall");
                telemetry.addLine("press 'y' to confirm!");
                telemetry.update();
            }

            if(gamepad1.y){
                telemetry.addLine("CONFIRMED");
                telemetry.update();
                break;
            }
        }

        //Initializing the IMU
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

        //Setting the direction of the motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    private void createVariableMap(){
        variableMap.put(Constants2020.BLUE_FLAG, this.isBlue);
        variableMap.put(Constants2020.WALL_FLAG, this.isWall);

        variableMap.put(Constants2020.BACK_LEFT_MOTOR,this.backLeft);
        variableMap.put(Constants2020.FRONT_LEFT_MOTOR,this.frontLeft);
        variableMap.put(Constants2020.BACK_RIGHT_MOTOR,this.backRight);
        variableMap.put(Constants2020.FRONT_RIGHT_MOTOR,this.frontRight);

        variableMap.put(Constants2020.TELEMETRY, this.telemetry);
        variableMap.put(Constants12907.IMU, this.imu);

        variableMap.put(Constants2020.WEBCAM, this.webcam);

        variableMap.put(Constants2020.ELAPSEDTIME, this.runtime);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("RunOpmode Entered");
        telemetry.update();

        try {

        initialize();

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
        telemetry.update();

        createVariableMap();

        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            //wobbleGoal.moveToTgtZone(isBlue, isWall);
        }

        //reset imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        telemetry.addLine("PROGRAM END");
        telemetry.update();


        } catch (Exception bad){
            telemetry.addData("EXCEPTION (from try catch):", bad.getMessage());
            bad.printStackTrace();
            telemetry.update();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
