package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.hardware.manip.Intake;


@Autonomous(group="Main")
public class RedStorage extends LinearOpMode {

    /* Declare OpMode members. */
    private DriveTrain dt = new DriveTrain();
    private ElapsedTime     runtime = new ElapsedTime();
    private HardwareDevice webcam_1;
    private DcMotorEx[] motors;
    private BNO055IMU gyro;
    private int degreeOffset = 2;
    public Intake intake = new Intake();
    public DcMotorEx lift;
    public DcMotor spinner2;
    public Servo leftServo;
    public Servo rightServo;
    public Servo midServo;
    public Servo rampServo;
    public TouchSensor magLim;
    private int level = 3;
    public RevColorSensorV3 color2;

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
        intake.init(hardwareMap.get(DcMotor.class, "intake"));
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        spinner2 = hardwareMap.get(DcMotor.class, "spinner2");
        leftServo = hardwareMap.get(Servo.class, "pushServo");
        rightServo = hardwareMap.get(Servo.class,"platServo");
        midServo = hardwareMap.get(Servo.class, "midServo");
        rampServo = hardwareMap.get(Servo.class, "rampServo");
        magLim = hardwareMap.get(TouchSensor.class, "magLim");
        color2 = hardwareMap.get(RevColorSensorV3.class,"color2");

        dt.initMotors(motors);
        dt.initGyro(gyro);
        waitForStart();

        //Auto Commands
        while(opModeIsActive()){
        }
    }
    public void strafeUntilMarker(double distanceIn, int velocity, boolean isRunning){
        int ticks = (int)(-distanceIn/(Math.PI*4)*515*1.1);
        dt.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dt.setDrivetrainPositions((int)ticks, (int)-ticks,(int)-ticks, (int)ticks);
        dt.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.setDrivetrainVelocity(velocity);

        ElapsedTime runtime = new ElapsedTime();
        boolean run = true;
        runtime.reset();
        while(!seesMarker()&&dt.fr.isBusy() && isRunning){
            telemetry.addData("Sees Marker: ", seesMarker());
            telemetry.addData("Distance: ", color2.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        if(seesMarker()){
            dt.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //Returns true when it sees the marker
    public boolean seesMarker(){
        int benchmarkDist = 2;

        return color2.getDistance(DistanceUnit.INCH)<benchmarkDist;
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

    //Lift commands
    public void liftToLevel(int level){
        if(level == 1){
            lift.setTargetPosition(350);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(600);
            dt.driveDistance(1,200,opModeIsActive());
        }
        //lift level 2 position on x
        else if(level == 2){
            lift.setTargetPosition(800);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(600);
        }
        //lift level 3 position on y
        else if(level ==3) {
            lift.setTargetPosition(1150);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(600);
            dt.driveDistance(-3,200,opModeIsActive());
        }
        //Default to lowest position
        else if(level == 0) {
            lift.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(400);
        }

        while(lift.isBusy()&&opModeIsActive()){
            telemetry.addData("Running lift to level ", level);
            telemetry.update();
        }
    }

    //Deposit command
    public void deposit(){
        leftServo.setPosition(0);
        rightServo.setPosition(.55);
        runtime.reset();
        while(runtime.milliseconds()<1500&&opModeIsActive()){
            telemetry.addLine("Depositing");
            telemetry.update();
        }
        leftServo.setPosition(.55);
        rightServo.setPosition(0);
    }
}