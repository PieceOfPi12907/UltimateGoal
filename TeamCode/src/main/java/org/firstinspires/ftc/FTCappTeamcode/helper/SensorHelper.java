package org.firstinspires.ftc.teamcode.helper;

        import android.graphics.Color;

        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

        import java.util.Locale;


public class SensorHelper {
    /*
    Not needed color sensing
    public boolean isYellow (ColorSensor color){
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        int red = (int) (color.red() * SCALE_FACTOR);
        int green = (int) (color.green()*SCALE_FACTOR);
        if (red<300&&green<300){
            return false;
        }
        else{
            return true;
        }*/

    public boolean isWhite(ColorSensor color,  Telemetry telemetry) {
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        final double GREEN_VALUE_FOR_WHITE = 100;
        final double RED_VALUE_FOR_WHITE = 100;
        final double BLUE_VALUE_FOR_WHITE = 100;



        Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                (int) (color.green() * SCALE_FACTOR),
                (int) (color.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Alpha", color.alpha());
        telemetry.addData("Red  ", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue ", color.blue());
        telemetry.addData("Hue", hsvValues[0]);

        telemetry.update();


        /*
        unused telemetry
        int red = (int) (color.red() * SCALE_FACTOR);
        int green = (int) (color.green()*SCALE_FACTOR);
        telemetry.addData("Red: ", red);
        telemetry.addData("Green: ", green);
        telemetry.update();
        */

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if (color.green()>GREEN_VALUE_FOR_WHITE && color.red()>RED_VALUE_FOR_WHITE && color.blue()>BLUE_VALUE_FOR_WHITE){
            return true;
        }
        else {
            return false;


        }
    }

}

