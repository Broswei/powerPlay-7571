package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;

@TeleOp (group = "DriveTest")
public class AdvancedDrive extends Robot{

private boolean yButton2Toggle=false;

        boolean isSlow = false;
        boolean isFast = false;
        //Changes behavior when intaking
        boolean hasClaw = false;
        int liftZero = 0;
        double gyroOffset = 0;
        boolean pressed=false;
        int tracker = 0;
        double[] positions = new double[]{-4.5, -3, -1.5};
        boolean stackyWacky = false;
        boolean lastMaggy = false;

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

        isSlow = gamepad1.right_trigger>.01;
        hasClaw = gamepad2.right_trigger>.01;

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
            lift.targetDistance(34,2000);
        }
        else if (gamepad2.x){
            lift.targetDistance(24, 2000);
        }
        else if (gamepad2.b){
            lift.targetDistance(16,2000);
        }
        else if (gamepad2.left_trigger>.01){
            lift.targetDistance(5,2000);
        }
        else if (gamepad2.left_bumper){
            lift.targetDistance(3.5, 2000);
        }
        else if (hasClaw){
            lift.targetDistance(0, 1000);

        }
        else{
            lift.targetDistance(0,2000);
        }

        if (gamepad2.right_trigger > 0.01){
            claw.setPosition(0);
        }
        else{
            claw.setPosition(0.4);
        }

        if (magLim.isPressed()){
            if (!lastMaggy){
                lift.lift.setVelocity(0);
                lift.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            lastMaggy = true;
        } else {
            lastMaggy = false;
        }

        telemetry.addLine("Servo Position: " + claw.getPosition());
        telemetry.addLine("Slides Position: "+  lift.lift.getCurrentPosition());
        telemetry.addLine("Vertical Orientation: " + gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES));
        telemetry.update();
    }
}