package org.firstinspires.ftc.SkystoneTeamcode.utillities;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.SkystoneTeamcode.helper.Constants12907;
import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SkystoneDetection {

    public void moveToSkystoneInner(DcMotor pFrontLeft, DcMotor pFrontRight, DcMotor pBackLeft, DcMotor pBackRight, NavigationHelper pNavigate, Telemetry pTelemetry){
        pNavigate.navigate(22, Constants12907.Direction.LEFT,0,0.25,pBackLeft,pBackRight,pFrontRight,pFrontLeft,null,pTelemetry);
    }
}
