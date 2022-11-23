package org.firstinspires.ftc.teamcode.lib.util;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Global variables used throughout classes
 * these need to be used by more than one class at a time
 */

public class GlobalVars {

    public static double worldXPosition = 0.0;
    public static double worldYPosition = 0.0;
    public static double worldAngle_rad = 0.0;

    public static double currentXTicks = 0;
    public static double currentYTicks = 0;


    public static double wxRelative = 0.0;
    public static double wyRelative = 0.0;

    public static double movement_x = 0;
    public static double movement_y = 0;
    public static double movement_turn = 0;

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double aTarget = 0;

    //use these to make more accurate control names
    // i.e. if(spinIntake) rather than if(gamepad1.a)
    public static Gamepad mainGp, auxGp;

    public final static double mTolerance = 1;
    public final static double aTolerance = 1;

    //PIDx
    public static double xKp = 0.07;
    public static double xKi = 0;
    public static double xKd = 0;

    //PIDy
    public static double yKp = 0.07;
    public static double yKi = 0;
    public static double yKd = 0;

    //PIDa
    public static double aKp = 0.03;
    public static double aKi = 0;
    public static double aKd = 0;

}
