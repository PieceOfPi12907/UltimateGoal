package org.firstinspires.ftc.SkystoneTeamcode.tester;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "Sensor Test", group = "Sensor")

public class ColorSensorTester extends LinearOpMode {


    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has an IR proximity sensor which is used to calculate distance and an RGB color sensor.
     * <p>
     * There will be some variation in the values measured depending on whether you are using a
     * V3 color sensor versus the older V2 and V1 sensors, as the V3 is based around a different chip.
     * <p>
     * For V1/V2, the light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     * <p>
     * For V3, the distance sensor as configured can handle distances between 0.25" (~0.6cm) and 6" (~15cm).
     * Any target closer than 0.25" will dislay as 0.25" and any target farther than 6" will display as 6".
     * <p>
     * Note that the distance sensor function of both chips is built around an IR proximity sensor, which is
     * sensitive to ambient light and the reflectivity of the surface against which you are measuring. If
     * very accurate distance is required you should consider calibrating the raw optical values read from the
     * chip to your exact situation.
     * <p>
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     * <p>
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     * <p>
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.
     */
    //ColorSensor sensorColor;
    //DistanceSensor sensorDistance;
    DistanceSensor quarryDistance;

    @Override
    public void runOpMode() {

        // get a reference to the color sensor.
        //sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance_left");

        // get a reference to the distance sensor that shares the same name.
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance_left");

        quarryDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance_quarry");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        /*float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 1;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
*/
        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        if (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
*/

            telemetry.addData("DISTANCE: ", quarryDistance.getDistance(DistanceUnit.INCH));

            // send the info back to driver station using telemetry function.
           /* telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            //telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red: ", sensorColor.red());
            telemetry.addData("Green: ", sensorColor.green());
            telemetry.addData("Blue:  ", sensorColor.blue());

            telemetry.addData("HUE: ", hsvValues[0]);
            telemetry.addData("SATURATION: ", hsvValues[1]);
            telemetry.addData("VALUE: ", hsvValues[2]);*/

            telemetry.update();
            try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            /*if (hsvValues[0] > 35) {
                telemetry.addLine("STONE COLOR: YELLOW");
                telemetry.update();
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                telemetry.addLine("STONE COLOR: BLACK");
                telemetry.update();
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
            /*relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });*/
        }
    }
}

