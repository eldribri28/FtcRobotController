package org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.GobildaMotorEnum.YELLOWJACKET_1150;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.GobildaMotorEnum.YELLOWJACKET_223;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public final class Constants {

    public static final double MATCH_START_DELAY = 0.0; // Delay in s

    /*
    Turret
     */
    public static final double LAUNCH_HEIGHT = 0.406;
    public static final double TARGET_HEIGHT = 0.880;
    public static final double FLYWHEEL_DIAMETER_METERS = .096;

    public static final double ACCELERATION_DUE_TO_GRAVITY = 9.81;
    public static final double TURRET_GEAR_RATIO = 1.872340426;
    public static final double TURRET_TICKS_PER_DEGREE = ( ( TURRET_GEAR_RATIO * YELLOWJACKET_223.getPPR() ) / 360 );
    public static final double LAUNCHER_MOTOR_IDLE_VELOCITY = 2200;
    public static final double TURRET_ENCODER_MAX_RIGHT = 1.4;
    public static final double TURRET_ENCODER_MAX_LEFT = 354.4;
    public static final double  TURRET_ENCODER_DEGREES_TO_ACTUAL = (TURRET_ENCODER_MAX_LEFT - TURRET_ENCODER_MAX_RIGHT) / 180;
    public static final double TURRET_ENCODER_ROBOT_POSE_ZERO = (TURRET_ENCODER_MAX_LEFT - TURRET_ENCODER_MAX_RIGHT) / 2;
    public static final double SYSTEM_LATENCY = 0.100; // Time it takes to start a launch in seconds
    /*
    AprilTag
     */
    public static final long AGED_DATA_LIMIT_MILLISECONDS = 50;
    public static final long TURRET_AGE_DATA_LIMIT_MILLISECONDS = 100;
    public static final double MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL = 150;
    public static final Pose2D TURRET_ROBOT_POSE_OFFSET = new Pose2D(DistanceUnit.METER, -0.040, 0, AngleUnit.RADIANS, 0);
    public static final Pose2D RED_GOAL_POSE = new Pose2D(DistanceUnit.METER, -1.680, 1.600, AngleUnit.RADIANS, 0);
    public static final Pose2D BLUE_GOAL_POSE = new Pose2D(DistanceUnit.METER, -1.680, -1.780, AngleUnit.RADIANS, 0);
    /*
    Drive System
     */
    public static final double DRIVE_MOTOR_POWER = 0.8;
    public static final double DRIVE_MOTOR_MULTIPLIER = 0.3;
    public static final double MAX_DRIVE_VELOCITY_MPS = 2; // Meters per second
    public static final double AUTON_DRIVE_VELOCITY_MPS = 2.2; // Meters per second
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
    public static final double TURRET_PID_P = 0.015;
    public static final double TURRET_PID_I = 0.000;
    public static final double TURRET_PID_D = 0.140;
    /*
    Launch Servo
     */
    public static final double LAUNCH_GATE_OPEN = 0.80;
    public static final double LAUNCH_GATE_CLOSE = 0.36;
    /*
    Color indicator control
     */
    public static final int MINIMUM_COLOR_HIT_COUNT_TO_CHANGE = 3;
    /*
    Intake
     */
    public static final double INTAKE_POWER_IN = -1;
    public static final double INTAKE_POWER_OUT = 1;
    public static final double INTAKE_NO_POWER = 0;
    public static final double INTAKE_IDLE_POWER = 0.2;

}
