package org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.GobildaMotorEnum.YELLOWJACKET_1150;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.GobildaMotorEnum.YELLOWJACKET_312;


public final class Constants {


    /*
    Camera Settings
     */
    public static final int CAMERA_GAIN = 1;
    public static final long CAMERA_EXPOSURE = 5; // In milliseconds

    /*
    Turret
     */
    public static final double LAUNCH_HEIGHT = 0.355;
    public static final double TARGET_HEIGHT = 1.175;
    public static final double FLYWHEEL_DIAMETER_METERS = .096;
    public static final double VELOCITY_TRANSFER_EFFICIENCY = 0.41;
    public static final double ACCELERATION_DUE_TO_GRAVITY = 9.81;
    public static final double TURRET_GEAR_RATIO = 5.11;
    public static final double TURRET_TICKS_PER_DEGREE = ( ( 5.11 * YELLOWJACKET_312.getPPR() ) / 360 );

    /*
    AprilTag
     */
    public static final long AGED_DATA_LIMIT_MILLISECONDS = 500;
    public static final long TURRET_AGE_DATA_LIMIT_MILLISECONDS = 50;
    public static final double MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL = 50;
    public static final double MANUAL_LAUNCH_MOTOR_VELOCITY_START = 1500;
    public static final double MANUAL_LAUNCH_MOTOR_VELOCITY_INCREMENT = 100;
    public static final double MANUAL_TURRET_MOTOR_MULTIPLIER = 0.5;

    /*
    Drive System
     */
    public static final double DRIVE_MOTOR_MULTIPLIER = 0.3;
    public static final double MAX_DRIVE_VELOCITY_MPS = 2; // Meters per second
    public static final double AUTON_DRIVE_VELOCITY_MPS = 1; // Meters per second
    public static final double WHEEL_DIAMETER = 0.104; // in meters
    public static final double WHEEL_CIRCUMFERENCE = (Math.PI * WHEEL_DIAMETER);
    public static final double ENCODER_TICKS_PER_METER = (YELLOWJACKET_1150.getPPR() / WHEEL_CIRCUMFERENCE);
    public static final double MAX_DRIVE_VELOCITY_TICKS_PER_SECOND = MAX_DRIVE_VELOCITY_MPS * ENCODER_TICKS_PER_METER;
    public static final double AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND = AUTON_DRIVE_VELOCITY_MPS * ENCODER_TICKS_PER_METER;
    public static final double MAX_DRIVE_VELOCITY_METER_PER_SECOND = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND / ENCODER_TICKS_PER_METER;
    public static final double ROTATION_ACCURACY = 1;
    
    /*
    Drive PID
     */
    public static final double ROTATEPID_P = 4.0;
    public static final double ROTATEPID_I = 0.0;
    public static final double ROTATEPID_D = 0.0;
    public static final double DRIVEPID_P = 4.0;
    public static final double DRIVEPID_I = 0.0;
    public static final double DRIVEPID_D = 0.0;

    /*
    Turret PID
     */
    public static final double TURRET_PID_P = 0.035;
    public static final double TURRET_PID_I = 0.0;
    public static final double TURRET_PID_D = 0.0;

    /*
    Launch Servo
     */
    public static final double LAUNCH_SERVO_UP = 0.0;
    public static final double LAUNCH_SERVO_DOWN = 0.65;
}
