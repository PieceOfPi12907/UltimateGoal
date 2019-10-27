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

}
