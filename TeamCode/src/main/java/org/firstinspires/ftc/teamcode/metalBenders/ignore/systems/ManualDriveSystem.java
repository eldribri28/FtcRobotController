package org.firstinspires.ftc.teamcode.metalBenders.ignore.systems;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVE_MOTOR_MULTIPLIER;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.types.SystemPriorityEnum;

public class ManualDriveSystem extends AbstractSystem {

    @Override
    public SystemPriorityEnum getSystemPriority() {
        return SystemPriorityEnum.PRIORITY_1;
    }

    @Override
    protected void start() {
        //do nothing
    }

    @Override
    protected void process() {
        double axial = -getHardwareManager().getGamepad().left_stick_y;
        double lateral = getHardwareManager().getGamepad().left_stick_x;
        double yaw = getHardwareManager().getGamepad().right_stick_x * 1.1;
        YawPitchRollAngles orientation = getHardwareManager().getImu().getRobotYawPitchRollAngles();
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);
        double cosHeading = Math.cos(botHeading);
        double sinHeading = Math.sin(botHeading);
        double rotX = lateral * cosHeading - axial * sinHeading;
        double rotY = lateral * sinHeading + axial * cosHeading;

        //calculate power
        double leftFrontPower = (rotY + rotX + yaw);
        double rightFrontPower = ((rotY - rotX) - yaw);
        double leftRearPower = ((rotY - rotX) + yaw) ;
        double rightRearPower = ((rotY + rotX) - yaw);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftRearPower   /= max;
            rightRearPower  /= max;
        }

        getHardwareManager().getLeftFrontMotor().setPower(leftFrontPower * DRIVE_MOTOR_MULTIPLIER);
        getHardwareManager().getRightFrontMotor().setPower(rightFrontPower * DRIVE_MOTOR_MULTIPLIER);
        getHardwareManager().getLeftRearMotor().setPower(leftRearPower * DRIVE_MOTOR_MULTIPLIER);
        getHardwareManager().getRightRearMotor().setPower(rightRearPower * DRIVE_MOTOR_MULTIPLIER);

        addTelemetry("Motor (Left Front)", leftFrontPower);
        addTelemetry("Motor (Right Front)", rightFrontPower);
        addTelemetry("Motor (Left Rear)", leftRearPower);
        addTelemetry("Motor (Right Rear)", leftRearPower);

        maybeResetImu();
    }

    @Override
    protected void shutdown() {
        //do nothing
    }

    private void maybeResetImu() {
        if (getHardwareManager().getGamepad().dpad_up) {
            getHardwareManager().getImu().resetYaw();
        }
    }
}
