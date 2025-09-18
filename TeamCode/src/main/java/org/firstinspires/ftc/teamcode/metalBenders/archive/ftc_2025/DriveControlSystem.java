package org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Map;

public class DriveControlSystem extends AbstractSystem{

    public DriveControlSystem(HardwareManagementSystem hardwareManagementSystem, Map<String, String> telemetry) {
        super(hardwareManagementSystem, telemetry);
    }

    @Override
    protected void process() {
        double axial = -hardwareManagementSystem.getGamepad().left_stick_y;
        double lateral = hardwareManagementSystem.getGamepad().left_stick_x;
        double yaw = hardwareManagementSystem.getGamepad().right_trigger - hardwareManagementSystem.getGamepad().left_trigger;
        YawPitchRollAngles orientation = hardwareManagementSystem.getImu().getRobotYawPitchRollAngles();
        double botHeading = -orientation.getYaw(AngleUnit.RADIANS);
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

        hardwareManagementSystem.getLeftFrontMotor().setPower(leftFrontPower);
        hardwareManagementSystem.getRightFrontMotor().setPower(rightFrontPower);
        hardwareManagementSystem.getLeftRearMotor().setPower(leftRearPower);
        hardwareManagementSystem.getRightRearMotor().setPower(rightRearPower);

        addTelemetry("Motor (Left Front)", "%.2f", leftFrontPower);
        addTelemetry("Motor (Right Front)", "%.2f", rightFrontPower);
        addTelemetry("Motor (Left Rear)","%.2f", leftRearPower);
        addTelemetry("Motor (Right Rear)", "%.2f", leftRearPower);

        resetIMU();
    }

    private void resetIMU() {
        if (hardwareManagementSystem.getGamepad().back) {
            hardwareManagementSystem.getImu().resetYaw();
        }
    }
}
