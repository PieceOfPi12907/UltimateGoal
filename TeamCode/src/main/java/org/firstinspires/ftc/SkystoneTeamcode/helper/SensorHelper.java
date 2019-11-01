package org.firstinspires.ftc.SkystoneTeamcode.helper;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class SensorHelper {

    public boolean isYellow (ColorSensor color){
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 1;

        int red = (int) (color.red() * SCALE_FACTOR);
        int green = (int) (color.green()*SCALE_FACTOR);
        if (red>100&&green>100){
            return true;
        }
        else{
            return false;
        }
    }
    public int getBlackBlock (ColorSensor colorLeft, ColorSensor colorRight){
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        int redR = (int)(colorRight.red());
        int greenR = (int) (colorRight.green());
        int redL = (int)(colorLeft.red());
        int greenL = (int)(colorLeft.green());
        if((redL > 1000) && (redR > 1000) && (greenL > 1000) && (greenR > 1000)){
            return 3;

        }else if((redR>redL) && (greenR>greenL)){
            return 1;

        }else{
            return 2;
        }

    }

}
