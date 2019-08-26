package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helper.MotorHelper;

//making sure the program doesn't show up:
@Disabled

//Version 10.19.18 (fixed switching & made dif drive ways into methods)
@TeleOp(name = "Boom TeleOp 12907", group = "teleop")
public class BoomTeleOp12907 extends LinearOpMode {

    int     buttonCount;
    boolean bButton, aButtonPressed, bButtonPressed;

    //Naming the motors
/*    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor lift;

    */Servo sweeperDump; /*

    DcMotor boom;
    //DcMotor sweeperDump;

    Servo latch;
    Servo markerDropper;
    BNO055IMU imu = null;

    final double LATCH_EXTENDED_POSITION = 0.8;
    final double LATCH_RETRACTED_POSITION = 0.2;
*/

    /**
     * Initializes motors from the hardware map
     */
    public void initialize() {
        //Configuration of the motors
        /*frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        lift = hardwareMap.get(DcMotor.class, "lift");*/

        sweeperDump = hardwareMap.get(Servo.class, "sweeperDump");

        //boom = hardwareMap.get(DcMotor.class, "boom");

        //sweeperDump = hardwareMap.get(DcMotor.class, "sweeperDump");


       /* latch = hardwareMap.get(Servo.class, "latch");
        markerDropper = hardwareMap.get(Servo.class, "markerDropper");
        //Setting the direction of the motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);*/

       //boom.setDirection(DcMotorSimple.Direction.FORWARD);

       //sweeperDump.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //This method is for Tank Drive (a)
    /*public void tankDrive(double powerRight, double powerLeft) {
        //joystick goes from -1 to +1, so we will negate it
        powerLeft = -gamepad1.left_stick_y;
        powerRight = -gamepad1.right_stick_y;

        //Since using quadratic drive, scale factor implements not full power - for more control
        final double POWER_SCALE_FACTOR = 0.9;

        //frontLeft.setPower(powerLeft*0.75);
        //backLeft.setPower(powerLeft*0.75);
        //frontRight.setPower(powerRight*0.75);
        //backRight.setPower(powerRight*0.75);

        //Quadratic Drive: the power is set to x squared
        if (powerLeft != 0) {
            frontLeft.setPower(Math.pow(powerLeft * POWER_SCALE_FACTOR , 3)/Math.abs(powerLeft * POWER_SCALE_FACTOR));
            backLeft.setPower(Math.pow(powerLeft * POWER_SCALE_FACTOR, 3)/Math.abs(powerLeft * POWER_SCALE_FACTOR));
        } else {
            frontLeft.setPower(powerLeft);
            backLeft.setPower(powerLeft);
        }

        if (powerRight != 0) {
            frontRight.setPower(Math.pow(powerRight * POWER_SCALE_FACTOR, 3)/Math.abs(powerRight * POWER_SCALE_FACTOR));
            backRight.setPower(Math.pow(powerRight * POWER_SCALE_FACTOR, 3)/Math.abs(powerRight * POWER_SCALE_FACTOR));
        } else {
            frontRight.setPower(powerRight);
            backRight.setPower(powerRight);
        }

    }

    //This method is for Arcade Drive (b)
    public void arcadeDrive(double powerRight, double powerLeft){
        //joystick goes from -1 to +1, so we will negate it
        double powerDrive = -gamepad1.left_stick_y;
        double powerTurn = gamepad1.right_stick_x;

        powerRight = Range.clip(powerDrive + powerTurn, -1.0, 1.0);
        powerLeft = Range.clip(powerDrive - powerTurn, -1.0, 1.0);
        frontLeft.setPower(powerLeft*0.75);
        backLeft.setPower(powerLeft*0.75);
        frontRight.setPower(powerRight*0.75);
        backRight.setPower(powerRight*0.75);
    }
*/
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        //INITIALIZING BOOM MOTOR
        //boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //boom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double powerLeft = 0;
        double powerRight = 0;
        double powerLift = 0;

        double powerSweeperDumper = 1;

        double powerDrive;
        double powerTurn;
        boolean driveMode=false;
        boolean driveInput=false;
        MotorHelper motorHelper = new MotorHelper();
        while (opModeIsActive()) {

            if (gamepad2.right_bumper) {
                sweeperDump.setPosition(0.8);
            }
            if (gamepad2.left_bumper) {
                sweeperDump.setPosition(0.2);
            }


            /*//Sweeper Dumping Motor:
            powerSweeperDumper = -gamepad2.right_stick_y;
            double finalPowerSweeperDumper = powerSweeperDumper * 0.7;
            sweeperDump.setPower(finalPowerSweeperDumper);
            telemetry.addData("Sweeper Dumper Power", finalPowerSweeperDumper);
            telemetry.update();*/

            //Y = increases
            /*if (gamepad2.y)
                if(!aButtonPressed)
                {
                    buttonCount += 1;
                    aButtonPressed = true;

                    if(powerSweeperDumper < 0.9) {
                        powerSweeperDumper = powerSweeperDumper + 0.2;
                    }
                    sweeperDump.setPower(powerSweeperDumper);
                }
                else {}
            else
                aButtonPressed = false;

            if (gamepad2.a)
                if(!bButtonPressed)
                {
                    bButton = !bButton;
                    bButtonPressed = true;

                    if(powerSweeperDumper > 0.1) {
                        powerSweeperDumper = powerSweeperDumper - 0.2;
                    }
                    sweeperDump.setPower(powerSweeperDumper);


                }
                else {}
            else
                bButtonPressed = false;*/

            /*telemetry.addData("Mode", "running");
            telemetry.addData("A button Count", buttonCount);
            telemetry.addData("B button", bButton);
            telemetry.addData("Power of Sweeper Dumper: ", powerSweeperDumper);
            telemetry.update(); */







/*
            //GAMEPAD 1:
            //driveMode a is a tank drive, while driveMode b is an arcade style drive
            while (!driveInput) {
                if (gamepad1.a) {
                    driveMode = true;
                    driveInput=true;
                }
                if (gamepad1.b) {
                    driveMode = false;
                    driveInput=true;
                }
            }
            if (driveMode) {
                tankDrive(powerRight, powerLeft);
                telemetry.addData("Drive Mode: ", "Tank Drive");
                telemetry.update();

            }
            if (!driveMode) {
                arcadeDrive(powerRight, powerLeft);
                telemetry.addData("Drive Mode: ", "Arcade Drive");
                telemetry.update();
            }

            //GAMEPAD 2:
            //servo latching -->
            if (gamepad2.x){
                latch.setPosition(LATCH_EXTENDED_POSITION);
            }
            if (gamepad2.a){
                latch.setPosition(LATCH_RETRACTED_POSITION);
            }

            //lift up and down -->
            powerLift = -gamepad2.right_stick_y;
            lift.setPower(powerLift);
 */





            //boom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          //  boom.setPower(0);

            //Boom motor extend and retract -->
           /* if (gamepad2.a){

                    telemetry.addData("Which pressed: ", "a");
                telemetry.addData("Boom Current Position before moving: ", boom.getCurrentPosition());
                    telemetry.update();

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e){
                    e.printStackTrace();
                }

                boom.setTargetPosition(boom.getCurrentPosition() + 100);
                boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                boom.setPower(0.5);

               while(boom.isBusy()){

                }

                telemetry.addData("Boom Current Position: ", boom.getCurrentPosition());
                telemetry.update();
                
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e){
                    e.printStackTrace();
                }

            } */



           /* if (gamepad2.b){

                telemetry.addData("Which pressed: ", "b");
                telemetry.addData("Boom Current Position before moving: ", boom.getCurrentPosition());
                telemetry.update();

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e){
                    e.printStackTrace();
                }

                boom.setTargetPosition(boom.getCurrentPosition() - 100);
                boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                boom.setPower(0.5);

                while(boom.isBusy()){

                }

                telemetry.addData("Boom Current Position: ", boom.getCurrentPosition());
                telemetry.update();

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e){
                    e.printStackTrace();
                }
                */


              /*  telemetry.addData("Which pressed: ", "b");
                telemetry.update();

               try {
                    Thread.sleep(2000);
                } catch (InterruptedException e){
                    e.printStackTrace();
                }

                boom.setTargetPosition(500);
                boom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                boom.setPower(-0.5);

                while(boom.isBusy()){
                    telemetry.addData("Boom Current Position: ", boom.getCurrentPosition());
                    telemetry.update();
                }
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e){
                    e.printStackTrace();
                }

            }*/
/*





 */
            //dropping the marker -->
            /*if(gamepad2.y) {
                markerDropper.setPosition(LATCH_EXTENDED_POSITION);
            }
            if(gamepad2.b){
                markerDropper.setPosition(LATCH_RETRACTED_POSITION);
            } */

            /*telemetry.addData("Power Left: ", powerLeft);
            telemetry.addData("Power Right: ", powerRight);
            telemetry.update();*/
            idle();



        }
    }

}
