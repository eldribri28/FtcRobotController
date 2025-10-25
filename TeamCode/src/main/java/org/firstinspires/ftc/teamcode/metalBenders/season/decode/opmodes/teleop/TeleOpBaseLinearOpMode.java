package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.GREEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.NONE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_NANO;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVE_MOTOR_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_ANGLE_SERVO_POSITION_INCREMENT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_LAUNCH_MOTOR_VELOCITY_INCREMENT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_LAUNCH_MOTOR_VELOCITY_START;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_TURRET_MOTOR_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_NANO;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Map;

public abstract class TeleOpBaseLinearOpMode extends LinearOpMode {
    private HardwareManager hardwareManager;
    private ArtifactColorEnum intakeArtifactColor = ArtifactColorEnum.NONE;
    private ArtifactColorEnum launcherArtifactColor = ArtifactColorEnum.NONE;
    private final PIDController turretBearingPid = new PIDController(0.05, 0.005, 0.05);
    private double targetDistance = 0;
    private AprilTagEngine aprilTagEngine;
    private boolean isManualLaunchOverrideActive = false;
    private double manualLaunchVelocity = MANUAL_LAUNCH_MOTOR_VELOCITY_START;
    abstract AprilTagEnum getTargetAprilTag();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread aprilTagEngineThread = new Thread(aprilTagEngine);
        waitForStart();
        resetRuntime();
        aprilTagEngineThread.start();
        hardwareManager.getLaunchServo().setPosition(0.7);
        while (opModeIsActive()) {
            updateRuntime();
            telemetry.addData("Manual Launch Controls Active", isManualLaunchOverrideActive);
            telemetry.addData("Target name", getTargetAprilTag().name());
            telemetry.addData("Motif detected", aprilTagEngine.getArtifactMotif().name());
            telemetry.addData("Intake artifact color", intakeArtifactColor.name());
            telemetry.addData("Launcher artifact color", launcherArtifactColor.name());
            telemetry.addData("Target distance", targetDistance);
            telemetry.addData("Turret Limit Switch Left Pressed", hardwareManager.getLimitSwitchLeft().isPressed());
            telemetry.addData("Turret Limit Switch Right Pressed", hardwareManager.getLimitSwitchRight().isPressed());

            Map<String,String> aprilTagTelemetry = aprilTagEngine.getTelemetry();
            for(String key : aprilTagTelemetry.keySet()) {
                String value = aprilTagTelemetry.get(key);
                telemetry.addData(key, value);
            }

            drive();
            intakeOrRejectArtifact();
            clearArtifactFromLaunch();
            setArtifactColors();
            setManualLaunchOverride(aprilTagEngineThread);
            launch();
            resetIMU();
            telemetry.update();
        }
        aprilTagEngineThread.interrupt();
        aprilTagEngine.teardown();
        telemetry.update();
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager);
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
                "Runtime",
                "%02.0f:%02.0f",
                totalSeconds / 60,
                totalSeconds % 60);
    }

    private void drive() {
        double axial = -hardwareManager.getGamepad1().left_stick_y;
        double lateral = -hardwareManager.getGamepad1().left_stick_x;
        double yaw = hardwareManager.getGamepad1().right_stick_x * 1.1;
        YawPitchRollAngles orientation = hardwareManager.getImu().getRobotYawPitchRollAngles();
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);
        double cosHeading = Math.cos(botHeading);
        double sinHeading = Math.sin(botHeading);
        double rotX = lateral * cosHeading - axial * sinHeading;
        double rotY = lateral * sinHeading + axial * cosHeading;

        //calculate power
        double leftFrontPower = (rotY + rotX + yaw);
        double rightFrontPower = ((rotY - rotX) - yaw);
        double leftRearPower = ((rotY - rotX) + yaw);
        double rightRearPower = ((rotY + rotX) - yaw);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        hardwareManager.getLeftFrontMotor().setPower(leftFrontPower * DRIVE_MOTOR_MULTIPLIER);
        hardwareManager.getRightFrontMotor().setPower(rightFrontPower * DRIVE_MOTOR_MULTIPLIER);
        hardwareManager.getLeftRearMotor().setPower(leftRearPower * DRIVE_MOTOR_MULTIPLIER);
        hardwareManager.getRightRearMotor().setPower(rightRearPower * DRIVE_MOTOR_MULTIPLIER);

        telemetry.addData("Motor (Left Front)", "%.2f", leftFrontPower);
        telemetry.addData("Motor (Right Front)", "%.2f", rightFrontPower);
        telemetry.addData("Motor (Left Rear)", "%.2f", leftRearPower);
        telemetry.addData("Motor (Right Rear)", "%.2f", rightRearPower);
    }

    private void resetIMU() {
        if (hardwareManager.getGamepad1().dpad_right) {
            hardwareManager.getImu().resetYaw();
        }
    }

    private void intakeOrRejectArtifact() {
        if (hardwareManager.getGamepad1().left_trigger > 0) {
            hardwareManager.getIntakeMotor().setPower(1);
        } else if (hardwareManager.getGamepad1().left_bumper) {
            hardwareManager.getIntakeMotor().setPower(-1);
        } else {
            hardwareManager.getIntakeMotor().setPower(0);
        }
    }

    private void setManualLaunchOverride(Thread aprilTagEngineThread) {
        if(hardwareManager.getGamepad2().dpad_left && hardwareManager.getGamepad2().x) {
            hardwareManager.getGamepad2().rumble(250);
            isManualLaunchOverrideActive = true;
            hardwareManager.getLauncherMotor().setVelocity(manualLaunchVelocity);
            aprilTagEngineThread.interrupt();
        } else if (hardwareManager.getGamepad1().dpad_left && hardwareManager.getGamepad1().x) {
            hardwareManager.getGamepad1().rumble(250);
            isManualLaunchOverrideActive = false;
            aprilTagEngineThread.start();
        }
    }

    private void launch() {
        if(isManualLaunchOverrideActive) {
            manualLaunch();
        } else {
            autoLaunch();
        }
    }
    private void manualLaunch() {
        if(hardwareManager.getGamepad2().dpad_up) {
            manualLaunchVelocity += MANUAL_LAUNCH_MOTOR_VELOCITY_INCREMENT;
        }
        if(hardwareManager.getGamepad2().dpad_down) {
            manualLaunchVelocity -= MANUAL_LAUNCH_MOTOR_VELOCITY_INCREMENT;
        }
        hardwareManager.getLauncherMotor().setVelocity(manualLaunchVelocity);
        if(Math.abs(hardwareManager.getGamepad2().left_stick_x) > 0
                && canRotateTurret(hardwareManager.getGamepad2().left_stick_x)) {
            hardwareManager.getTurretMotor().setPower(
                    hardwareManager.getGamepad2().left_stick_x * MANUAL_TURRET_MOTOR_MULTIPLIER);
        } else {
            hardwareManager.getTurretMotor().setPower(0);
        }
        if (Math.abs(hardwareManager.getGamepad2().right_stick_y) > 0) {
            double currentPosition = hardwareManager.getAngleServo().getPosition();
            hardwareManager.getAngleServo().setPosition(
                    currentPosition + MANUAL_ANGLE_SERVO_POSITION_INCREMENT);
        }
        if(hardwareManager.getGamepad2().right_trigger > 0) {
            autoLaunchArtifact();
            autoIntakeArtifact();
        }
    }

    private void autoLaunch() {
        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            processTargetDetection(aprilTagEngine.getBlueTargetDetection());
        } else {
            processTargetDetection(aprilTagEngine.getRedTargetDetection());
        }
    }

    private boolean canRotateTurret(double input) {
        boolean rightSwitchPressed = hardwareManager.getLimitSwitchRight().isPressed();
        boolean leftSwitchPressed = hardwareManager.getLimitSwitchRight().isPressed();
        if(input == 0 || (!rightSwitchPressed && ! leftSwitchPressed)) {
            return true;
        } else if (input > 0 && !hardwareManager.getLimitSwitchRight().isPressed()) {
            return true;
        } else if (input < 0 && !hardwareManager.getLimitSwitchLeft().isPressed()) {
            return true;
        }
        return false;
    }

    private void setArtifactColors() {
        intakeArtifactColor = getArtifactColor(hardwareManager.getIntakeColorSensor(), "Intake");
        launcherArtifactColor = getArtifactColor(hardwareManager.getLaunchColorSensor(), "Launcher");
    }

    private ArtifactColorEnum getArtifactColor(NormalizedColorSensor colorSensor, String sensorName) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;

        ArtifactColorEnum artifactColorEnum;
        if (blue > green && red > 0.2 && blue > 0.3) {
            artifactColorEnum = PURPLE;
        } else if (green > red && green > blue && green > 0.3) {
            artifactColorEnum = GREEN;
        } else {
            artifactColorEnum = NONE;
        }
        telemetry.addData(sensorName + " - RGB", "%6.3f %6.3f %6.3f", red, green, blue);
        return artifactColorEnum;
    }

    private void processTargetDetection(AprilTagDetection targetDetection) {
        if(targetDetection != null) {
            long detectionAge = targetDetection.frameAcquisitionNanoTime - System.nanoTime();
            telemetry.addData("Target detection age(millisecond)", Math.abs(detectionAge));

            if (detectionAge < AGED_DATA_LIMIT_NANO) {
                if (detectionAge < TURRET_AGE_DATA_LIMIT_NANO && canRotateTurret(targetDetection.ftcPose.bearing)) {
                    hardwareManager.getTurretMotor().setPower(turretBearingPid.calculate(1, targetDetection.ftcPose.bearing));
                } else {
                    hardwareManager.getTurretMotor().setPower(0);
                }
                targetDistance = targetDetection.ftcPose.range / 39.37;

                double flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                telemetry.addData("flywheel RPM", flywheelRPM);

                LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult1(flywheelRPM, targetDistance);
                if (launchResult != null) {
                    setLaunchAngle(launchResult.getLaunchAngle());
                }

                double targetRPM = Math.round(((targetDistance / 1.670) * 800.0) + 1900.0);
                //hardwareManager.getLauncherMotor().setVelocity((targetRPM / 60.0) * 28.0);
                telemetry.addData("target RPM", targetRPM);

                if (hardwareManager.getGamepad1().right_trigger > 0) {
                    hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
                    if (Math.abs(((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0) - targetRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL) {
                        autoLaunchArtifact();
                        autoIntakeArtifact();
                    }
                } else {
                    hardwareManager.getLauncherMotor().setVelocity(0);
                }
            } else {
                //stop rotating turret if detection is too old
                hardwareManager.getTurretMotor().setPower(0);
            }
        } else {
            //stop rotating turret if there is no detection
            hardwareManager.getTurretMotor().setPower(0);
        }
    }

    private void autoLaunchArtifact() {
        hardwareManager.getLaunchServo().setPosition(0);
        sleep(100);
        hardwareManager.getLaunchServo().setPosition(0.7);
        sleep(150);
    }

    private void clearArtifactFromLaunch() {
        boolean clear = false;
        if(isManualLaunchOverrideActive && hardwareManager.getGamepad2().right_bumper) {
            clear = true;
        } else if (!isManualLaunchOverrideActive && hardwareManager.getGamepad1().right_bumper) {
            clear = true;
        }
        if (clear) {
            hardwareManager.getLauncherMotor().setPower(0.4);
            sleep(100);
            autoLaunchArtifact();
        }
    }

    private void autoIntakeArtifact() {
        hardwareManager.getIntakeMotor().setPower(1);
        sleep(25);
        hardwareManager.getIntakeMotor().setPower(0);
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((62.0 - launchAngle) / 35.0));
        hardwareManager.getAngleServo().setPosition(positionValue);
        telemetry.addData("launchAngle", launchAngle);
    }
}

