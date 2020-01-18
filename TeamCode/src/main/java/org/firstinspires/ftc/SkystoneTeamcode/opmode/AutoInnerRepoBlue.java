/*package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helper.MotorHelper;

@Autonomous(name = "AutonomousTest_Encoders", group = "autonomous")
public class AutoInnerOneBlockRepoBlue extends LinearOpMode {
    //Naming the motors
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    MotorHelper motorHelper;

    public void initialize() {
        //Configuration of the motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        //setting the directions of the motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
            motorHelper.movingWithEncoders(frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor,
                    powerRight, powerLeft,
                    targetPositionRight, targetPositionLeft,
                    timeoutS, telemetry);
        }
    }
}
*/

package org.firstinspires.ftc.SkystoneTeamcode.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Parking;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.Repositioning;
import org.firstinspires.ftc.SkystoneTeamcode.utillities.SkystoneDetection;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

@Disabled
//Adding Source Code to GitHub

public class AutoInnerRepoBlue {

    public void playProgram(HashMap<String, Object> pVariableMap) {

        Telemetry telemetry = (Telemetry) pVariableMap.get(Constants12907.TELEMETRY);
        boolean isStoneRepo = false;

        try {

            NavigationHelper navigationHelper = new NavigationHelper();
            SkystoneDetection skystoneDetection = new SkystoneDetection();
            Repositioning repositioning = new Repositioning();

            Constants12907.SkystonePosition skystonePosition = (Constants12907.SkystonePosition) pVariableMap.get(Constants12907.SKY_POSITION);
            ElapsedTime runtime = (ElapsedTime) pVariableMap.get(Constants12907.ELAPSEDTIME);

            DcMotor backLeft = (DcMotor) pVariableMap.get(Constants12907.BACK_LEFT_MOTOR);
            DcMotor frontLeft = (DcMotor) pVariableMap.get(Constants12907.FRONT_LEFT_MOTOR);
            DcMotor backRight = (DcMotor) pVariableMap.get(Constants12907.BACK_RIGHT_MOTOR);
            DcMotor frontRight = (DcMotor) pVariableMap.get(Constants12907.FRONT_RIGHT_MOTOR);

            Servo repositioningRight = (Servo) pVariableMap.get(Constants12907.RIGHT_REPOSITIONING_SERVO);
            Servo repositioningLeft = (Servo) pVariableMap.get(Constants12907.LEFT_REPOSITIONING_SERVO);

            BNO055IMU imu = (BNO055IMU) pVariableMap.get(Constants12907.IMU);

            Boolean isBlue = (Boolean) pVariableMap.get(Constants12907.BLUE_FLAG);
            Boolean isOuter = (Boolean) pVariableMap.get(Constants12907.OUTER_FLAG);


            repositioning.doAngleRepositioning (frontLeft, frontRight, backLeft, backRight, navigationHelper, imu, telemetry, isBlue, isOuter, isStoneRepo, repositioningRight, repositioningLeft, runtime);

        } catch (Exception bad) {
            telemetry.addData("EXCEPTION:", bad.toString());
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}



