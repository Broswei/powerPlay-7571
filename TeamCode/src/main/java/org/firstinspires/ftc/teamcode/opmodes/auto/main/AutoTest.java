package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//vision libraries
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain;


@Autonomous(group="Main")
public class AutoTest extends LinearOpMode {

    /* Declare OpMode members. */
    private DriveTrain dt = new DriveTrain();
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareDevice webcam_1;
    private DcMotorEx[] motors;
    private BNO055IMU gyro;
    public DcMotorEx lift;
    public Servo claw;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        motors = new DcMotorEx[]{hardwareMap.get(DcMotorEx.class, "fl"), hardwareMap.get(DcMotorEx.class, "fr"), hardwareMap.get(DcMotorEx.class, "bl"), hardwareMap.get(DcMotorEx.class, "br")};
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        //distR = hardwareMap.get(RevColorSensorV3.class, "distanceR");
        //distL = hardwareMap.get(RevColorSensorV3.class, "distanceL");


        dt.initMotors(motors);
        dt.initGyro(gyro);
        waitForStart();

        //Auto Commands

        while (opModeIsActive()) {

        }


    }
}

