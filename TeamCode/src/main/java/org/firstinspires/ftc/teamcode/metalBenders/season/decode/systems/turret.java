package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_ENCODER_DEGREES_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_ENCODER_ROBOT_POSE_ZERO;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_TICKS_PER_DEGREE;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class turret {

    private final static PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);

    public static double getTurretAngleFromEncoder(long encoderValue, long encoderZero) {
        return Math.toRadians((encoderValue - encoderZero) / TURRET_TICKS_PER_DEGREE);
    }

    public static double adjustTurretPowerNearStop(double turretAbsoluteAngle, double setPower) {
        turretAbsoluteAngle = Math.toDegrees(turretAbsoluteAngle);
        if ((setPower < 0 && turretAbsoluteAngle < -70) || (setPower > 0 && turretAbsoluteAngle > 70)) {
            return setPower * (Math.abs(turretAbsoluteAngle - 70) * 0.05);
        } else {
            return setPower;
        }
    }

    public static double rotateTurret(double turretError, boolean limitSwitchRight, boolean limitSwitchLeft, double turretAngle, boolean robotPoseSetFlag) {
        double turretAngleDeg = Math.toDegrees(turretAngle);
        if (robotPoseSetFlag) {
            if (turretAngleDeg < 0 && turretAngleDeg - turretError < -87) {
                turretError = 0;
            } else if (turretAngleDeg > 0 && turretAngleDeg + turretError > 87) {
                turretError = 0;
            }
            double setPower = turretError * 0.7; //turretBearingPid.calculate(0, turretError) * 0.8;
            if (!canRotateTurret(setPower, limitSwitchRight, limitSwitchLeft, turretAngle, turretError)) {
                setPower = 0;
            }
            return setPower;
        } else {
            return 0;
        }
    }

    public static long setTurretEncoderToMotorEncoderOffset(long turretMotorEncoder, double turretAngleEncoder) {
        double currentTurretEncoderAdjustedPosition = AngleUnit.normalizeDegrees(TURRET_ENCODER_ROBOT_POSE_ZERO - Math.toDegrees(turretAngleEncoder));
        double currentTurretActualPosition = AngleUnit.normalizeDegrees(currentTurretEncoderAdjustedPosition / TURRET_ENCODER_DEGREES_TO_ACTUAL);
        return turretMotorEncoder - (long)(currentTurretActualPosition * TURRET_TICKS_PER_DEGREE);
    }

    public static boolean canRotateTurret(double input, boolean limitSwitchRight, boolean limitSwitchLeft, double turretAngle, double turretError) {
        if ((input < 0 && limitSwitchRight) || (input < 0 && turretAngle < -(Math.PI / 2))) {
            return false;
        } else if ((input > 0 && limitSwitchLeft) || (input > 0 && turretAngle > (Math.PI / 2))) {
            return false;
        //} else if (Math.abs(turretAngle + turretError) > 88 || Math.abs(turretAngle) > 88) {
        //    return false;
        } else {
            return true;
        }
    }

    public static int calculateTurretErrorEncoderPosition(double turretError, double turretAngle, long turretMotorEncoder) {
        double turretAngleDeg = Math.toDegrees(turretAngle);
        double turretErrorTicks = turretError * TURRET_TICKS_PER_DEGREE;
        double turretSetEncoder = turretMotorEncoder + turretErrorTicks;
        double turretAngleRequired = -turretAngleDeg + turretError;
        if (turretAngleRequired < -86) { turretSetEncoder = -86 * TURRET_TICKS_PER_DEGREE; }
        if (turretAngleRequired > 88) { turretSetEncoder = 88 * TURRET_TICKS_PER_DEGREE; }
        return (int) turretSetEncoder;
    }

}
