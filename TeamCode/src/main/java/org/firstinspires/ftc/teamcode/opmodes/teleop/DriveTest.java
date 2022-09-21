package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;

@TeleOp (group = "DriveTest")
public class DriveTest extends Robot{


        //boolean test=true;

        boolean isSlow = false;
        boolean isFast = false;

        //Changes behavior when intaking

        boolean isIntaking = false;
        int liftZero = 0;
        double gyroOffset = 0;

//private ElapsedTime timer=new ElapsedTime();


    @Override
    public void init(){
        super.init();

        isAuto(false);



        }

    @Override
    public void start(){

        }

    @Override
    public void loop(){
        super.loop();

        //Sets if we are intaking or not
        isIntaking = gamepad2.right_trigger>.1;

        isSlow = gamepad1.left_trigger>.01 || gamepad1.right_trigger>.01;

        if(gamepad1.y){
            gyroOffset = dt.getGyroRotation(AngleUnit.RADIANS);
        }

        if(gamepad1.a){
            isFast = true;
        }else{
            isFast = false;
        }

        //Strafe drive, slows down when intaking
        if(gamepad1.left_trigger>.1){
            dt.manualControl(gamepad1,isSlow);
        }else{
            dt.fieldOrientedControl(gamepad1, isSlow, isFast, gyroOffset);
        }


        telemetry.update();
    }
}