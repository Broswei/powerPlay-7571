package org.firstinspires.ftc.teamcode.lib.hardware.base;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDx;
import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDy;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.aTarget;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.aTolerance;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.auxGp;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.currentXTicks;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.currentYTicks;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.mTolerance;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.mainGp;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldXPosition;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldYPosition;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.wxRelative;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.wyRelative;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.xTarget;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.yTarget;

/*import org.firstinspires.ftc.teamcode.lib.hardware.skystone.Clamp;
import org.firstinspires.ftc.teamcode.lib.hardware.skystone.Depositor;
import org.firstinspires.ftc.teamcode.lib.hardware.skystone.Elevator;
import org.firstinspires.ftc.teamcode.lib.hardware.skystone.FoundationMover;*/
//import org.firstinspires.ftc.teamcode.lib.hardware.skystone.Intake;

/**
 * main robot class
 * all opmodes will extend this class
 */
//@TeleOp
public class RobotGyro extends OpMode{

    public static boolean isAuto = true;

    public DecimalFormat df = new DecimalFormat("###.###");

    private DcMotorEx[] motors;

    public DriveTrain dt = new DriveTrain();

    public static ElapsedTime timer = new ElapsedTime();

    public static Telemetry telemetry;

    @Override
    public void init() {
        motors = new DcMotorEx[]{hardwareMap.get(DcMotorEx.class, "fl"), hardwareMap.get(DcMotorEx.class, "fr"), hardwareMap.get(DcMotorEx.class, "bl"), hardwareMap.get(DcMotorEx.class, "br")};
        //stores gamepads in global variables
        if(!isAuto){
            getGamepads(gamepad1, gamepad2);

        }

        dt.initMotors(motors);
        dt.initGyro(hardwareMap.get(BNO055IMU.class, "imu"));

    }

    @Override
    public void init_loop(){



        telemetry.addLine("wx: " + worldXPosition);
        telemetry.addLine("wy: " + worldYPosition);
        telemetry.update();


        timer.reset();
    }

    @Override
    public void loop() {

        if(isAuto) {
            dt.update();
        } else {
            dt.applyMovement();
        }


        telemetry.addLine("wx: " + worldXPosition);
        telemetry.addLine("wy: " + worldYPosition);

        telemetry.update();



   /* packet.put("wx", worldXPosition);
    packet.put("wy", worldYPosition);
    packet.put("wa", Math.toDegrees(worldAngle_rad));
    packet.put("auto", auto);
    packet.put("autostate", autoState);
    packet.put("robot state", roboState);*/

        //dashboard.sendTelemetryPacket(packet);

    }

    /**
     * stores gamepads in global variables
     * @param main main gamepad, used for driving the robot base
     * @param aux auxiliary gamepad, used for driving the scoring mechanisms
     */
    public void getGamepads(Gamepad main, Gamepad aux){

        mainGp = main;
        auxGp = aux;

    }

}
