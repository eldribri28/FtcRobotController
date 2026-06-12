package org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Tuning {

    public static boolean tuningEnabled = false; // Are we in Tuning Mode?
    /*
    LAUNCHER TUNING
    */
    public static PIDFCoefficients LAUNCH_VELO_PID = new PIDFCoefficients(150, 0, 0, 3);
    public static double MAX_LAUNCH_ANGLE = 53;
    public static double MIN_LAUNCH_ANGLE = 33;
    public static double HOOD_SERVO_MAX_VALUE = 0.65;
    public static double HOOD_SERVO_MIN_VALUE = 0.30;

    public static double VELOCITY_TRANSFER_EFFICIENCY = 0.36;
    /*
    TURRET TUNING
     */
    public static double TURRET_ERROR_SMOOTHING_FACTOR = 0.08; // How much of the last error should be blended into the current error 100% = 1.0
    public static PIDFCoefficients TURRET_VELO_PID = new PIDFCoefficients(6.4, 0, 6.2, 6.6);
    /*
    Camera Settings
    */
    public static int CAMERA_GAIN = 5;
    public static long CAMERA_EXPOSURE = 7; // In milliseconds

}
