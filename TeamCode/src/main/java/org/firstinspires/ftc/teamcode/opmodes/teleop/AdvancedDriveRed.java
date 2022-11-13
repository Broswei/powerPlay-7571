package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.hardware.manip.Lift;

@TeleOp (group = "DriveTest")
public class AdvancedDriveRed extends Robot{

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
        }
        else {
            dt.fieldOrientedControl(gamepad1, isSlow, isFast, gyroOffset);
        }

        if (gamepad2.y){
            lift.pullDistance(34,1250);
        }
        else if (gamepad2.x){
            lift.pullDistance(24, 1250);
        }
        else if (gamepad2.b){
            lift.pullDistance(16,1250);
        }
        else{
            lift.pullDistance(0, 1250);
        }

        if (gamepad2.right_trigger > 0.01){
            claw.setPosition(1);
        }
        else{
            claw.setPosition(0.5);
        }



        telemetry.addLine("Servo Position: " + claw.getPosition());
        telemetry.addLine("Slides Position: "+  lift.lift.getCurrentPosition());
        telemetry.update();
    }
}