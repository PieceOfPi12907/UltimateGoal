package org.firstinspires.ftc.SkystoneTeamcode.tester;

        import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
//@TeleOp(name = "dumper Test New",group = "teleop")
public class DumperTestNew extends LinearOpMode {
    Servo dumperClampInsideServo;
    Servo dumperClampOutsideServo;
    DcMotor dumperMotor;
    DcMotor slideMotor;
    boolean isInsideClampUp = true;
    boolean isOutsideClampUp = false;

    final double DUMPER_CLAMP_INSIDE_UP = 0.5;
    final double DUMPER_CLAMP_INSIDE_DOWN = 0.05;
    final double DUMPER_CLAMP_OUTSIDE_UP = 0.3;
    final double DUMPER_CLAMP_OUTSIDE_DOWN = 0.9;
    ElapsedTime b_time = new ElapsedTime();
    ElapsedTime y_time = new ElapsedTime();
    ElapsedTime x_time = new ElapsedTime();
    ElapsedTime a_time = new ElapsedTime();
    ElapsedTime right_bumper_time = new ElapsedTime();
    ElapsedTime a_time_two = new ElapsedTime();
    private void initialize(){
        dumperMotor = hardwareMap.get(DcMotor.class,"dumperMotor");
        slideMotor = hardwareMap.get(DcMotor.class,"slideMotor");
        dumperClampInsideServo = hardwareMap.get(Servo.class,"dumperClampInsideServo");
        dumperClampOutsideServo = hardwareMap.get(Servo.class,"dumperClampOutsideServo");

    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.right_trigger>=0.1){
                dumperMotor.setPower(Math.pow(gamepad2.right_trigger,2));
            }
            if(gamepad2.left_trigger>=0.1){
                dumperMotor.setPower(-Math.pow(gamepad2.left_trigger,2));
            }
            if(gamepad2.right_trigger<0.1){
                dumperMotor.setPower(0);
            }
            if(gamepad2.left_trigger<0.1){
                dumperMotor.setPower(0);
            }
            if(gamepad2.left_stick_y>=0){
                slideMotor.setPower(-Math.pow(gamepad2.left_stick_y,2));
            }
            if(gamepad2.left_stick_y<0){
                slideMotor.setPower(Math.pow(gamepad2.left_stick_y,2));
            }
            if(gamepad2.x && x_time.seconds()>0.25){
                x_time.reset();
                if(!isInsideClampUp){
                    dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_UP);
                    isInsideClampUp = true;
                }else if(isInsideClampUp){
                    dumperClampInsideServo.setPosition(DUMPER_CLAMP_INSIDE_DOWN);
                    isInsideClampUp = false;
                }
            }

            if(gamepad2.b && b_time.seconds()>0.25){
                b_time.reset();
                if(!isOutsideClampUp){
                    dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_UP);
                    isOutsideClampUp = true;
                }else if(isOutsideClampUp){
                    dumperClampOutsideServo.setPosition(DUMPER_CLAMP_OUTSIDE_DOWN);
                    isOutsideClampUp = false;
                }
            }
        }

    }

}

