package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_DRIVE_VELOCITY_METER_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_TICKS_PER_DEGREE;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

public abstract class TurretBearing {

    abstract AprilTagEnum getTargetAprilTag();

    /*
    Calculate turret bearing from otos, apriltag bearing, and turretPosition
     */
    public static double calculateTurretAngleFromOtos(double turretEncoderPosition, double h) {

        double turretPosition = getTurretChassisOffset(turretEncoderPosition);
        if (turretPosition != 0) {
            return ( h + turretPosition + 360 ) % 360;
        } else {
            return 0;
        }

    }

    /*
    Get current turret angle
     */
    public static double getTurretChassisOffset(double turretEncoderPosition) {
        if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
            return Math.round(((( turretEncoderPosition + TURRET_LEFT_LIMIT_ENCODER_VALUE ) / TURRET_TICKS_PER_DEGREE ) + 360 ) % 360 );
        } else {
            return 0;
        }
    }





}
