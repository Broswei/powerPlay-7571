package org.firstinspires.ftc.teamcode.lib.hardware.manip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;

public class AngleAdjuster extends Subsystem {

    public DcMotorEx angleAdjuster;

    public static int[] angles = new int[]{0, 72, 120, 160, 180};


    public AngleAdjuster(){

    }

    public void init(DcMotorEx angleAdjuster) {

        this.angleAdjuster = angleAdjuster;

        this.angleAdjuster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.angleAdjuster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void update() {

    }


    public void setPower(double power){
        angleAdjuster.setPower(power);
    }

    @Override
    public void finishJob() {

    }

}



