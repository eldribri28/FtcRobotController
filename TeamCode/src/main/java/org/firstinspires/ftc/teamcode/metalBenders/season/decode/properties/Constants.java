package org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.GobildaMotorEnum.YELLOWJACKET_1150;

public final class Constants {

    public static final double DRIVE_MOTOR_MULTIPLIER = 0.5;
    /*
    Turret
     */
    public static final double LAUNCH_HEIGHT = 0.3;
    public static final double TARGET_HEIGHT = 1.175;
    public static final double FLYWHEEL_DIAMETER_METERS = .096;
    public static final double VELOCITY_TRANSFER_EFFICIENCY = 0.41;
    public static final double ACCELERATION_DUE_TO_GRAVITY = 9.81;
    /*
    AprilTag
     */
    public static final long MILLISECONDS_TO_NANOSECONDS = 1000000;
    public static final long AGED_DATA_LIMIT_NANO = 500 * MILLISECONDS_TO_NANOSECONDS;
    public static final long TURRET_AGE_DATA_LIMIT_NANO = 50 * MILLISECONDS_TO_NANOSECONDS;
    public static final double MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL = 150;
    public static final double MANUAL_LAUNCH_MOTOR_VELOCITY_START = 1500;
    public static final double MANUAL_LAUNCH_MOTOR_VELOCITY_INCREMENT = 100;
    public static final double MANUAL_TURRET_MOTOR_MULTIPLIER = 0.5;
    /*
    Drive System
     */
    public static final double MAX_DRIVE_VELOCITY = 1100; // Ticks Per Second
    public static final double MAX_ROTATE_VELOCITY = 1100; // Ticks per second
    public static final double WHEEL_DIAMETER = 0.104; // in meters
    public static final double WHEEL_CIRCUMFERENCE = (Math.PI * WHEEL_DIAMETER);
    public static final double ENCODER_TICKS_PER_METER = (YELLOWJACKET_1150.getPPR() / WHEEL_CIRCUMFERENCE);
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
}
