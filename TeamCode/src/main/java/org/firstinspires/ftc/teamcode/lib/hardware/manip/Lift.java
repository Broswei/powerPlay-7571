package org.firstinspires.ftc.teamcode.lib.hardware.manip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;

public class Lift extends Subsystem {

    public DcMotorEx lift;

    public static int[] angles = new int[]{0, 72, 120, 160, 180};


    public Lift(){

        this.lift = lift;

        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void pullDistance(double distanceIn, int velocity){

        
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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



