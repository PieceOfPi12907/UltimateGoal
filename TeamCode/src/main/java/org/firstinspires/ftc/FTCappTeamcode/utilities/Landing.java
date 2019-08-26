package org.firstinspires.ftc.teamcode.utilities;



import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.helper.MotorHelper;

import com.qualcomm.robotcore.hardware.Servo;


/* public class Landing {
    final double LIFT_ACTUATOR_POWER = 0.5;
    final int LIFT_ACTUATOR_ENCODER_POSITION = 1000;
    final double LATCH_SERVO_POSITION = 0.8;

    MotorHelper motorHelper = new MotorHelper();

    public void drop (DcMotor pLiftMotor, Servo pLatchServo){

        motorHelper.liftActuator(pLiftMotor, LIFT_ACTUATOR_POWER, LIFT_ACTUATOR_ENCODER_POSITION);
        motorHelper.latch(pLatchServo, LATCH_SERVO_POSITION);
    }
}
*/

public class Landing {

    final double LATCH_SERVO_POSITION = 0.8;
    MotorHelper motorHelper = new MotorHelper();
    public void drop (DcMotor pLiftMotor, Servo pLatchServo,  MotorHelper pie, Telemetry telemetry){
        double pLiftPower = 0.25;
        int pTargetPosition= 100;
        double timeoutS = 5250;
        motorHelper.liftActuatorInches(pLiftMotor,  pLiftPower,  pTargetPosition, timeoutS, telemetry);
        motorHelper.latch(pLatchServo, LATCH_SERVO_POSITION, telemetry);
    }


}
