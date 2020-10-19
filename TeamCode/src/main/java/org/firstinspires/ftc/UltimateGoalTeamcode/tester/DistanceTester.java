package org.firstinspires.ftc.UltimateGoalTeamcode.tester;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "DistanceTester", group = "Test Programs")
public class DistanceTester extends LinearOpMode {
    DistanceSensor distanceSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance sensor reading ", distanceSensor.getDistance(DistanceUnit.INCH));
        }
    }
    public void initialize () {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DS");
    }
}

