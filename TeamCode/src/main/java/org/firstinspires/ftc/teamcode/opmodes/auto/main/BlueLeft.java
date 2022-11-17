package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.hardware.manip.Lift;


@Autonomous(group="Main")
public class BlueLeft extends LinearOpMode {

    /* Declare OpMode members. */
    private DriveTrain dt = new DriveTrain();
    private ElapsedTime     runtime = new ElapsedTime();
    private HardwareDevice webcam_1;
    private DcMotorEx[] motors;
    private BNO055IMU gyro;
    private int degreeOffset = 2;
    public Lift lift = new Lift();
    public Servo claw;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private static final String[] LABELS = {
            "Duck",
            "Marker"
    };

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        motors = new DcMotorEx[]{hardwareMap.get(DcMotorEx.class, "fl"), hardwareMap.get(DcMotorEx.class, "fr"), hardwareMap.get(DcMotorEx.class, "bl"), hardwareMap.get(DcMotorEx.class, "br")};
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        lift.init(hardwareMap.get(DcMotorEx.class, "lift"));
        claw = hardwareMap.get(Servo.class, "claw");


        dt.initMotors(motors);
        dt.initGyro(gyro);
        claw.setPosition(1);

        waitForStart();

        //Auto Commands
        dt.strafeDistance(-65,1000,opModeIsActive());
        lift.pullDistance(34,1250);
        while(lift.lift.isBusy());
        dt.driveDistance(-3.5,500,opModeIsActive());
        claw.setPosition(0.5);
        sleep(1000);
        dt.driveDistance(3.5,500,opModeIsActive());
        lift.pullDistance(4.75,750);
        turnDegrees(178.5,750);
        dt.strafeDistance(-12,750,opModeIsActive());
        dt.driveDistance(-24,750,opModeIsActive());
        fang();
        sleep(1000);
        lift.pullDistance(10,500);
        while(lift.lift.isBusy()){
            dt.driveDistance(24,1000,opModeIsActive());
        }
        dt.strafeDistance(-12, 750,opModeIsActive());
        turnDegrees(-178.5,750);
        dt.driveDistance(-3.5,750, opModeIsActive());
        claw.setPosition(0.5);
        sleep(1000);
        dt.driveDistance(3.5,750,opModeIsActive());
        turnDegrees(88.5, 750);
        int parkingSpace = (int)(Math.random()*3+1);
        dt.driveDistance(18,750,opModeIsActive());
        if (parkingSpace == 1){
            dt.strafeDistance(24,750,opModeIsActive());
        }
        else if (parkingSpace == 3){
            dt.strafeDistance(-24,750,opModeIsActive());
        }







        while(opModeIsActive()){
        }
    }

    //Turn by degrees
    public void turnDegrees(double turnDegrees,int velocity){
        dt.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double offset;
        offset = dt.getGyroRotation(AngleUnit.DEGREES);
        int predictedTicks = (int)(turnDegrees*9.34579439252+2000);
        double correctedDegrees;


        correctedDegrees = offset + turnDegrees;
        if(correctedDegrees>180){
            correctedDegrees-=360;
        }
        if(correctedDegrees>0){
            dt.setDrivetrainPositions(-predictedTicks,predictedTicks,-predictedTicks,predictedTicks);
            dt.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
            dt.setDrivetrainVelocity(velocity);
            while(correctedDegrees-degreeOffset>dt.getGyroRotation(AngleUnit.DEGREES)&&opModeIsActive()){
                telemetry.addLine("Running left");
                telemetry.addData("gyro Target: ", correctedDegrees);
                telemetry.addData("gyro: ", dt.getGyroRotation(AngleUnit.DEGREES));
                telemetry.addData("Offset: ", offset);
                telemetry.update();
            }
        }else if(correctedDegrees<0){
            dt.setDrivetrainPositions(predictedTicks,-predictedTicks,predictedTicks,-predictedTicks);
            dt.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
            dt.setDrivetrainVelocity(velocity);
            while(correctedDegrees+degreeOffset<dt.getGyroRotation(AngleUnit.DEGREES)&&opModeIsActive()){
                telemetry.addLine("Running right");
                telemetry.addData("gyro: ", dt.getGyroRotation(AngleUnit.DEGREES));
                telemetry.addData("gyro Target: ", correctedDegrees);
                telemetry.addData("Offset: ", offset);
                telemetry.update();
            }

        }
        dt.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void fang(){
        claw.setPosition(1);
    }

    public void score(int level){
        if (level == 1){
            lift.pullDistance(14,1000);

        }
        else if (level == 2){
            lift.pullDistance(22, 750);

        }
        else{
            lift.pullDistance(32, 1250);

        }
    }





}