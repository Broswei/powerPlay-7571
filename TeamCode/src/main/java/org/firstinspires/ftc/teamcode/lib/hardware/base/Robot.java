package org.firstinspires.ftc.teamcode.lib.hardware.base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.hardware.manip.Lift;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.auxGp;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.mainGp;

/**
 * main robot class
 * all opmodes will extend this class
 */
//@TeleOp
public class Robot extends OpMode {

  public static boolean isAuto = true;

  public DecimalFormat df = new DecimalFormat("###.###");

  private DcMotorEx[] motors;

  public DriveTrain dt = new DriveTrain();
  public Lift lift = new Lift();
  public BNO055IMU gyro;
  public Servo claw;
  public HardwareDevice Webcam1;


  public static ElapsedTime timer = new ElapsedTime();

  @Override
  public void init() {

    motors = new DcMotorEx[]{hardwareMap.get(DcMotorEx.class, "fl"), hardwareMap.get(DcMotorEx.class, "fr"), hardwareMap.get(DcMotorEx.class, "bl"), hardwareMap.get(DcMotorEx.class, "br")};

    //stores gamepads in global variables
    if (!isAuto) {
      getGamepads(gamepad1, gamepad2);

    }

    dt.initMotors(motors);
    gyro = hardwareMap.get(BNO055IMU.class, "imu");
    lift.init(hardwareMap.get(DcMotorEx.class, "lift"));
    claw = hardwareMap.get(Servo.class, "claw");
    Webcam1 = hardwareMap.get(HardwareDevice.class, "Webcam1");

  }

  @Override
  public void init_loop() {
    dt.initGyro(gyro);
    telemetry.update();

    timer.reset();
  }

  @Override
  public void loop() {

    if (isAuto) {
      dt.update();
    } else {
      dt.applyMovement();
    }


    telemetry.addLine("Gyro: " + dt.getGyroRotation(AngleUnit.RADIANS));
    telemetry.update();

  }

  /**
   * stores gamepads in global variables
   *
   * @param main main gamepad, used for driving the robot base
   * @param aux  auxiliary gamepad, used for driving the scoring mechanisms
   */
  public void getGamepads(Gamepad main, Gamepad aux) {

    mainGp = main;
    auxGp = aux;

  }


  /**
   * denotes whether or not the opmode currently running is auto or not
   *
   * @param isAuto true if this is auto, false if it is not auto
   */
  public void isAuto(boolean isAuto) {
    this.isAuto = isAuto;
  }

}