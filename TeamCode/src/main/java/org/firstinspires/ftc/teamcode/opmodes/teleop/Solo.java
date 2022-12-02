package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;

@TeleOp (group = "DriveTest")
public class Solo extends Robot{

private boolean yButton2Toggle=false;

        boolean isSlow = false;
        boolean isFast = false;
        //Changes behavior when intaking
        boolean isIntaking = false;
        int liftZero = 0;
        double gyroOffset = 0;
        boolean pressed=false;
        int tracker = 0;

private ElapsedTime timer=new ElapsedTime();

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
        isSlow = gamepad1.left_trigger>.01;

        if(gamepad1.right_bumper){
            gyroOffset = dt.getGyroRotation(AngleUnit.RADIANS);
        }
        if(gamepad1.a){
            isFast = true;
        }else{
            isFast = false;
        }

        //Strafe drive, slows down when intaking
        if(gamepad1.dpad_up){
            dt.manualControl(gamepad1,isSlow);
        }
        else {
            dt.fieldOrientedControl(gamepad1, isSlow, isFast, gyroOffset);
        }

        if (gamepad1.y){
            lift.targetDistance(34,2000);
        }
        else if (gamepad1.x){
            lift.targetDistance(24, 2000);
        }
        else if (gamepad1.b){
            lift.targetDistance(16,2000);
        }
        else{
            lift.targetDistance(0, 2000);
        }

        if (gamepad1.right_trigger > 0.01){
            claw.setPosition(0);
        }
        else{
            claw.setPosition(0.4);
        }



        telemetry.addLine("Servo Position: " + claw.getPosition());
        telemetry.addLine("Slides Position: "+  lift.lift.getCurrentPosition());
        telemetry.update();
    }
}