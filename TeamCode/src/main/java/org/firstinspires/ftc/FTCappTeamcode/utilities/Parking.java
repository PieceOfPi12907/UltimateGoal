package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.MotorHelper;

public class Parking {

    public void forwardCrater(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, MotorHelper pie, Telemetry telemetry) {
        double powerRight = 0.25;
        double powerLeft = 0.25;
        double targetPositionLeft = 70;
        double targetPositionRight = 70;
        double timeoutS = 45;
        pie.movingWithEncoders(frontRight, frontLeft, backRight, backLeft, powerRight, powerLeft, targetPositionRight, targetPositionLeft, timeoutS, telemetry);
    }
}
