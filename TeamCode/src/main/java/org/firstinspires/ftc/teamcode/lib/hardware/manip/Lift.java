package org.firstinspires.ftc.teamcode.lib.hardware.manip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;

public class Lift extends Subsystem {

    public DcMotorEx lift;

    public static int[] angles = new int[]{0, 72, 120, 160, 180};

    public Lift(){

    }


    public void init(DcMotorEx lift){

        this.lift = lift;

        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void targetDistance(double distanceIn, int velocity){

        int slideTicksPerRotation = 385;
        double ticks;

        ticks = (-distanceIn/(4.4)*slideTicksPerRotation);
        lift.setTargetPosition((int) ticks);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setVelocity(velocity);

    }

    @Override
    public void update() {

    }


    public void setPower(double power){
        lift.setPower(power);
    }

    @Override
    public void finishJob() {

    }

}



