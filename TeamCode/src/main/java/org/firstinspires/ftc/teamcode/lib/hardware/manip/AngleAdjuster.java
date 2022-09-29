package org.firstinspires.ftc.teamcode.lib.hardware.manip;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;

public class AngleAdjuster extends Subsystem {

    public DcMotor angleAdjuster;


    public AngleAdjuster(){

    }

    public void init(DcMotor angleAdjuster) {

        this.angleAdjuster = angleAdjuster;

        this.angleAdjuster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.angleAdjuster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.angleAdjuster.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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



