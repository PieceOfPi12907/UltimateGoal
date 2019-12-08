package org.firstinspires.ftc.SkystoneTeamcode.helper.Mechanum_Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Mechanum_Helper {
    public void moveForwardForTime(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, double power, double time){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds()<time) {
            fr.setPower(power);
            fl.setPower(power);
            br.setPower(power);
            bl.setPower(power);
        }
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public void strafeRightForTime (DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, double power, double time){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds()<time) {
            fr.setPower(-power);
            fl.setPower(power);
            br.setPower(power);
            bl.setPower(-power);
        }
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public void strafeLeftForTime (DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, double power, double time){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.seconds()<time) {
            fr.setPower(power);
            fl.setPower(-power);
            br.setPower(-power);
            bl.setPower(power);
        }

        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }


}
