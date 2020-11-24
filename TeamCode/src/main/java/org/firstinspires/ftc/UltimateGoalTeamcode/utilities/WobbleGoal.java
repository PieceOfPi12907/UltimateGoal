package org.firstinspires.ftc.UltimateGoalTeamcode.utilities;

import org.firstinspires.ftc.SkystoneTeamcode.helper.NavigationHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.helper.DetectionHelper;
import org.firstinspires.ftc.UltimateGoalTeamcode.opmode.UltimateAuto;

public class WobbleGoal {

    NavigationHelper navigater = new NavigationHelper();

    public void moveToTgtZone(DetectionHelper.RingPosition position, boolean isBlue, boolean isWall){
        if(isWall&&isBlue){
            if(position.equals(DetectionHelper.RingPosition.NONE)){
                navigater.navigate(70.75,1, 0,0.5, backLeft, backRights, frontRight,frontLeft, imu, telemetry);
            }
            if(position.equals(DetectionHelper.RingPosition.ONE)){

            }
            if(position.equals(DetectionHelper.RingPosition.FOUR)){

            }
        }
    }
}
