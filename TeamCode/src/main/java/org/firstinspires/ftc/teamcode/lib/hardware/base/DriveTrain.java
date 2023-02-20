package org.firstinspires.ftc.teamcode.lib.hardware.base;

//import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;


import static org.firstinspires.ftc.teamcode.lib.hardware.base.Robot.isAuto;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.aKd;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.aKi;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.aKp;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.aTarget;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.currentXTicks;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.movement_turn;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.movement_x;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.movement_y;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldXPosition;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldYPosition;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.xKd;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.xKi;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.xKp;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.xTarget;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.yKd;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.yKi;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.yKp;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.yTarget;

import dalvik.system.DelegateLastClassLoader;

/**
 * represents the robot's drivebase
 */
public class DriveTrain{

  public DcMotorEx fl, fr, bl, br;
  public BNO055IMU imu;

  private double[] motorPowers = new double[4];

  //the actual speed the robot is moving
  public static double xSpeed = 0;
  public static double ySpeed = 0;
  public static double turnSpeed = 0;

  double ticksPerRotation = 384.5;

  public static Telemetry telemetry;

  //last update time
  private long lastUpdateTime = 0;

  private double maxMotorPowerAuto = 0.4, maxMotorPowerManual = 1;

  //pid controller objects
  public static PIDController PIDx = new PIDController(xKp, xKi, xKd);
  public static PIDController PIDy = new PIDController(yKp, yKi, yKd);
  public static PIDController PIDa = new PIDController(aKp, aKi, aKd);

  public double ticks;
  public int currVelocity;
  public int targetVelocity;

  public DriveTrain(){

  }

  /**
   * initializes the motor objects and sets some necessary details
   * @param motors RevMotor array of the motors
   *               fl = 0
   *               fr = 1
   *               bl = 2
   *               br = 3
   */
  public void initMotors(DcMotorEx[] motors) {

    fl = motors[0];
    fr = motors[1];
    bl = motors[2];
    br = motors[3];

    fr.setDirection(DcMotorSimple.Direction.REVERSE);
    br.setDirection(DcMotorSimple.Direction.REVERSE);

    //setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);

    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

  }

  /**
   * initializes the imu objects
   * @param IMU imu objects
   */
  public void initGyro(BNO055IMU IMU){
    imu = IMU;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu.initialize(parameters);

  }
  public void initGyro(){
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu.initialize(parameters);
  }

    /**
     * used to control the drive base with a gamepad during teleop
     * @param gamepad gamepad object used to control the drivebase
     */
    public void manualControl(Gamepad gamepad, boolean isSlow){

        if(isSlow) {
          movement_x = Range.clip(-gamepad.left_stick_x, -0.4, 0.4);
          movement_y = Range.clip(gamepad.left_stick_y, -0.4, 0.4);
          movement_turn = Range.clip(-gamepad.right_stick_x, -0.3, 0.3);
        } else{
          movement_x = Range.clip(-gamepad.left_stick_x, -1, 1);
          movement_y = Range.clip(gamepad.left_stick_y, -1, 1);
          movement_turn = Range.clip(-gamepad.right_stick_x, -.6, .6);
        }
    }

    public void fieldOrientedControl(Gamepad gamepad, boolean isSlow, boolean isFast, double gyroOffset){
      if(isSlow) {
        movement_x = Range.clip(-gamepad.left_stick_x, -.2, .2);
        movement_y = Range.clip(gamepad.left_stick_y, -.2, .2);
        movement_turn = Range.clip(-gamepad.right_stick_x, -.4, .4);
      } else if(isFast){
        movement_x = Range.clip(-gamepad.left_stick_x, -1, 1);
        movement_y = Range.clip(gamepad.left_stick_y, -1, 1);
        movement_turn = Range.clip(-gamepad.right_stick_x, -1, 1);
      } else{
        movement_x = Range.clip(-gamepad.left_stick_x, -.5, .5);
        movement_y = Range.clip(gamepad.left_stick_y, -.5, .5);
        movement_turn = Range.clip(-gamepad.right_stick_x, -.6, .6);
      }

      double temp = movement_x*Math.cos(getGyroRotation(AngleUnit.RADIANS)-gyroOffset)+movement_y*Math.sin(getGyroRotation(AngleUnit.RADIANS)-gyroOffset);
      movement_y = -movement_x*Math.sin(getGyroRotation(AngleUnit.RADIANS)-gyroOffset)+movement_y*Math.cos(getGyroRotation(AngleUnit.RADIANS)-gyroOffset);
      movement_x = temp;
    }


  /**
   * sets the forward power for all 4 motors
   * (used only for testing)
   * @param power
   */
  public void setThrottle(double power){

      fl.setPower(power);
      fr.setPower(power);
      bl.setPower(power);
      br.setPower(power);

  }

  /**
   * sets the strafe power for all 4 motors
   * (used only for testing)
   * @param power
   */
  public void strafe(double power){

      fl.setPower(power);
      fr.setPower(-power);
      bl.setPower(-power);
      br.setPower(power);

  }

  public void driveDistance(double distanceIn, int velocity, boolean isRunning){
    ticks = (-distanceIn/(Math.PI*4)*ticksPerRotation);
    setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    setDrivetrainPositions((int)ticks);
    setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
    setDrivetrainVelocity(velocity);

    ElapsedTime runtime = new ElapsedTime();
    boolean run = true;
    runtime.reset();
    while(fr.isBusy() && isRunning){

    }
  }

  public void goBrrrr(double distanceIn, int velocity, boolean isRunning){
    ticks = (-distanceIn/(Math.PI*4)*ticksPerRotation);
    setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    setDrivetrainPositions((int)ticks);
    setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
    currVelocity = 0;
    setDrivetrainVelocity(currVelocity);
    ElapsedTime runtime = new ElapsedTime();
    while(fr.getVelocity() < velocity){
      runtime.reset();
      while(runtime.seconds() <= 0.2){

      }
      currVelocity += (velocity)/5;
      setDrivetrainVelocity(currVelocity);
    }
    while(fr.getVelocity() > 0){
      runtime.reset();
      while(runtime.seconds() <= 0.4){

      }
      currVelocity -= (targetVelocity)/5;
      setDrivetrainVelocity(currVelocity);
    }
  }

  public void strafeDistance(double distanceIn, int velocity, boolean isRunning){
    ticks = (-distanceIn/(Math.PI*4)*ticksPerRotation*1.1);
    setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    setDrivetrainPositions((int)ticks, (int)-ticks,(int)-ticks, (int)ticks);
    setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
    setDrivetrainVelocity(velocity);

    ElapsedTime runtime = new ElapsedTime();
    boolean run = true;
    runtime.reset();
    while(fr.isBusy() && isRunning){

    }
  }



  public void setDrivetrainMode(DcMotor.RunMode runMode){
    fr.setMode(runMode);
    fl.setMode(runMode);
    br.setMode(runMode);
    bl.setMode(runMode);
  }

  public void setDrivetrainPositions(int position){
    fr.setTargetPosition(position);
    fl.setTargetPosition(position);
    br.setTargetPosition(position);
    bl.setTargetPosition(position);
  }

  public void setDrivetrainPositions(int frPosition, int flPosition, int brPosition, int blPosition){
    fr.setTargetPosition(frPosition);
    fl.setTargetPosition(flPosition);
    br.setTargetPosition(brPosition);
    bl.setTargetPosition(blPosition);
  }

  public void setDrivetrainVelocity(int velocity){
    fr.setVelocity(velocity);
    fl.setVelocity(velocity);
    br.setVelocity(velocity);
    bl.setVelocity(velocity);
  }

  /**
   * applies the movement vars to the motors
   */
  public void applyMovement(){

    motorPowers[0] =  movement_y + movement_turn + movement_x;
    motorPowers[1] =  movement_y - movement_turn - movement_x;
    motorPowers[2] =  movement_y + movement_turn - movement_x;
    motorPowers[3] =  movement_y - movement_turn + movement_x;

    if(isAuto) {

        fl.setPower(Range.clip(motorPowers[0], -maxMotorPowerAuto, maxMotorPowerAuto));
        fr.setPower(Range.clip(motorPowers[1], -maxMotorPowerAuto, maxMotorPowerAuto));
        bl.setPower(Range.clip(motorPowers[2], -maxMotorPowerAuto, maxMotorPowerAuto));
        br.setPower(Range.clip(motorPowers[3], -maxMotorPowerAuto, maxMotorPowerAuto));
    } else {
        fl.setPower(Range.clip(motorPowers[0], -1, 1) * maxMotorPowerManual);
        fr.setPower(Range.clip(motorPowers[1], -1, 1) * maxMotorPowerManual);
        bl.setPower(Range.clip(motorPowers[2], -1, 1) * maxMotorPowerManual);
        br.setPower(Range.clip(motorPowers[3], -1, 1) * maxMotorPowerManual);
    }

  }

  /**
   * compares the current positions to their targets and calculates the movements for each movement direction/type
   */
  public void update(){

    movement_x = PIDx.getOutput(worldXPosition, xTarget);
    movement_y = PIDy.getOutput(worldYPosition, yTarget);
    movement_turn = -PIDa.getOutput(Math.toDegrees(worldAngle_rad), aTarget);

    applyMovement();

  }

  public void setPowers(double frp, double flp, double brp, double blp){
    fr.setPower(frp);
    fl.setPower(flp);
    br.setPower(brp);
    bl.setPower(blp);
  }

  public void setPowers(double rp, double lp){
    fr.setPower(rp);
    fl.setPower(lp);
    br.setPower(rp);
    bl.setPower(lp);
  }

  public void setAngleTarget(double angle){
      aTarget = angle;
  }

  /**
   * gets the rotation of the imu
   * @param unit angle unit, degrees or radians
   * @return rotation in unit as a double
   */
  public double getGyroRotation(AngleUnit unit) {
    return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, unit).firstAngle;
  }

  public String getTargetString(){
      return ("(" + xTarget + ", " + yTarget + ", " + aTarget + ")");
  }

}
