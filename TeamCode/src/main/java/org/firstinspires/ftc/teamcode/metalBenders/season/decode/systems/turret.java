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

    public static double rotateTurret(double error, boolean limitSwitchRight, boolean limitSwitchLeft, double turretAngle, boolean robotPoseSetFlag) {
        if (robotPoseSetFlag) {
            double setPower = turretBearingPid.calculate(0, error) * 1.0;
            if (!canRotateTurret(setPower, limitSwitchRight, limitSwitchLeft, turretAngle)) {
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

    public static boolean canRotateTurret(double input, boolean limitSwitchRight, boolean limitSwitchLeft, double turretAngle) {
        if ((input < 0 && limitSwitchRight) || (input < 0 && turretAngle < -(Math.PI / 2))) {
            return false;
        } else if ((input > 0 && limitSwitchLeft) || (input > 0 && turretAngle > (Math.PI / 2))) {
            return false;
        } else {
            return true;
        }
    }




}
