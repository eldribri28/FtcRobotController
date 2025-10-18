package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.AbstractBaseOpMode;

public class ManualDriveSystem extends AbstractSystem {

    public ManualDriveSystem(AbstractBaseOpMode baseOpMode) {
        super(baseOpMode);
    }

    @Override
    public SystemPriorityEnum getSystemPriority() {
        return SystemPriorityEnum.PRIORITY_1;
    }

    @Override
    protected void process() {
        double axial = -getBaseOpMode().getHardwareManager().getGamepad().left_stick_y;
        double lateral = getBaseOpMode().getHardwareManager().getGamepad().left_stick_x;
        double yaw = getBaseOpMode().getHardwareManager().getGamepad().right_stick_x * 1.1;
        YawPitchRollAngles orientation = getBaseOpMode().getHardwareManager().getImu().getRobotYawPitchRollAngles();
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

        getBaseOpMode().getHardwareManager().getLeftFrontMotor().setPower(leftFrontPower*0.25);
        getBaseOpMode().getHardwareManager().getRightFrontMotor().setPower(rightFrontPower*0.25);
        getBaseOpMode().getHardwareManager().getLeftRearMotor().setPower(leftRearPower*0.25);
        getBaseOpMode().getHardwareManager().getRightRearMotor().setPower(rightRearPower*0.25);

        getBaseOpMode().addTelemetry("Motor (Left Front)", "%.2f", leftFrontPower);
        getBaseOpMode().addTelemetry("Motor (Right Front)", "%.2f", rightFrontPower);
        getBaseOpMode().addTelemetry("Motor (Left Rear)","%.2f", leftRearPower);
        getBaseOpMode().addTelemetry("Motor (Right Rear)", "%.2f", leftRearPower);

        maybeResetImu();
    }

    private void maybeResetImu() {
        if (getBaseOpMode().getHardwareManager().getGamepad().dpad_up) {
            getBaseOpMode().getHardwareManager().getImu().resetYaw();
        }
    }
}
