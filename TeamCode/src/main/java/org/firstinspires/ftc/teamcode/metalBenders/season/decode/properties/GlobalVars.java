package org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties;

public class GlobalVars {

    /*
    BOOLEAN FLAGS
     */
    public static boolean CAMERA_GAIN_SET = false;

    /*
    Robot Positional Data
     */
    public static double BOT_HEADING_OFFSET = 0;
    public static double ROBOT_FIELD_X = 0;
    public static double ROBOT_FIELD_Y = 0;
    public static double ROBOT_FIELD_H = 0;

    /*
    Robot PID Vars
     */
    public static double ROTATE_PID_ACCUMULATED_ERROR = 0;
    public static double ROTATE_PID_LAST_TIME = System.currentTimeMillis();
    public static double ROTATE_PID_LAST_ERROR = 0;
    public static double DRIVE_PID_ACCUMULATED_ERROR = 0;
    public static double DRIVE_PID_LAST_TIME = System.currentTimeMillis();
    public static double DRIVE_PID_LAST_ERROR = 0;

    /*
    Turret Vars
     */
    public static long TURRET_LEFT_LIMIT_ENCODER_VALUE = 0;
    public static double TURRET_CHASSIS_OFFSET = 0;
    public static double IMU_YAW = 0;
    public static double TURRET_ANGLE = 0;
    public static double TURRET_IMU_OFFSET = 0;
    public static double IMU_OTOS_OFFSET;
    public static double TURRET_CURRENT_ENCODER;
}
