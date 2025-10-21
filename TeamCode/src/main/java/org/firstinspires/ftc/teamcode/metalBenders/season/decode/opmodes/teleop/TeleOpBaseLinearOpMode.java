package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.GREEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.NONE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVE_MOTOR_MULTIPLIER;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class TeleOpBaseLinearOpMode extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
    private HardwareManager hardwareManager;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private ArtifactMotifEnum artifactMotif = ArtifactMotifEnum.UNKNOWN;
    private ArtifactColorEnum intakeArtifactColor = ArtifactColorEnum.NONE;
    private ArtifactColorEnum launcherArtifactColor = ArtifactColorEnum.NONE;
    private final PIDController turretBearingPid = new PIDController(0.05, 0.005, 0.05);
    private double calculatedLaunchAngle = 0;
    private double targetDistance = 0;
    abstract AprilTagEnum getTargetAprilTag();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        resetRuntime();
        boolean isGainAndExposureSet = false;
        while (opModeIsActive()) {
            updateRuntime();
            telemetry.addData("Target name", getTargetAprilTag().name());
            telemetry.addData("Target id", getTargetAprilTag().getId());
            telemetry.addData("Motif detected", artifactMotif.name());
            if (!isGainAndExposureSet) {
                setExposureAndGain();
                isGainAndExposureSet = true;
            }
            drive();
            intakeOrRejectArtifact();
            setArtifactColors();
            //TODO REVIEW CODE WITH PRESTON
            detectAndProcessAprilTags();
            resetIMU();
            telemetry.update();
        }
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1);
        this.aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(3);
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareManager.getTurretCam());
        builder.addProcessor(aprilTagProcessor);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(640, 480));
        this.visionPortal = builder.build();
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
                "Runtime",
                "%02.0f:%02.0f",
                totalSeconds / 60,
                totalSeconds % 60);
    }

    private void setExposureAndGain() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (exposureControl != null && gainControl != null) {
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(5, TimeUnit.MILLISECONDS);
            gainControl.setGain(75);
        }
    }

    private void drive() {
        double axial = -hardwareManager.getGamepad().left_stick_y;
        double lateral = hardwareManager.getGamepad().left_stick_x;
        double yaw = hardwareManager.getGamepad().right_stick_x * 1.1;
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
        if (hardwareManager.getGamepad().dpad_up) {
            hardwareManager.getImu().resetYaw();
        }
    }

    private void intakeOrRejectArtifact() {
        //handle intake
        if (hardwareManager.getGamepad().left_trigger > 0) {
            hardwareManager.getIntakeMotor().setPower(1);
        } else if (hardwareManager.getGamepad().left_bumper) {
            hardwareManager.getIntakeMotor().setPower(-1);
        } else {
            hardwareManager.getIntakeMotor().setPower(0);
        }

        //handle low power launch
        if (hardwareManager.getGamepad().right_bumper) {
            hardwareManager.getLauncherMotor().setPower(0.1);
            sleep(100);
            autoLaunchBall();
        }
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
        telemetry.addData(sensorName + " - Color", artifactColorEnum.name());
        return artifactColorEnum;
    }

    private void detectAndProcessAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        if (currentDetections.isEmpty()) {
            hardwareManager.getTurretMotor().setPower(0);
        } else {
            for (AprilTagDetection detection : currentDetections) {
                if (detection != null) {
                    processDetection(detection);
                }
            }
        }
    }

    private void processDetection(AprilTagDetection detection) {
        AprilTagEnum aprilTagEnum = AprilTagEnum.findById(detection.id);
        if (aprilTagEnum != null) {
            if (getTargetAprilTag() == aprilTagEnum && detection.metadata != null) {
                processTargetDetection(detection);
            } else {
                if (artifactMotif == ArtifactMotifEnum.UNKNOWN) {
                    setArtifactMotif(aprilTagEnum);
                }
            }
        }
    }

    private void processTargetDetection(AprilTagDetection targetDetection) {
        hardwareManager.getTurretMotor().setPower(turretBearingPid.calculate(1, targetDetection.ftcPose.bearing));
        targetDistance = targetDetection.ftcPose.range / 39.37;

        double flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
        telemetry.addData("flywheel RPM", flywheelRPM);

        LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult1(flywheelRPM, targetDistance);
        if (launchResult != null) {
            calculatedLaunchAngle = launchResult.getLaunchAngle();
            setLaunchAngle(calculatedLaunchAngle);
        }

        double targetRPM = Math.round(((targetDistance / 1.670) * 800.0) + 1900.0);
        hardwareManager.getLauncherMotor().setVelocity((targetRPM / 60.0) * 28.0);
        telemetry.addData("target RPM", flywheelRPM);

        if (hardwareManager.getGamepad().right_trigger > 0) {
            hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
            if (Math.abs(((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0) - targetRPM) < 150.0) {
                if (launcherArtifactColor != ArtifactColorEnum.NONE) {
                    autoLaunchBall();
                }
                autoIntakeBall();
            }
        } else {
            hardwareManager.getLauncherMotor().setVelocity(0);
        }
    }

    private void autoLaunchBall() {
        hardwareManager.getLaunchServo().setPosition(0);
        sleep(100);
        hardwareManager.getLaunchServo().setPosition(0.7);
        sleep(100);
    }

    private void autoIntakeBall() {
        hardwareManager.getIntakeMotor().setPower(1);
        sleep(25);
        hardwareManager.getIntakeMotor().setPower(0);
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((62.0 - launchAngle) / 35.0));
        hardwareManager.getAngleServo().setPosition(positionValue);
        telemetry.addData("launchAngle", launchAngle);
    }

    private void setArtifactMotif(AprilTagEnum aprilTagEnum) {
        switch (aprilTagEnum) {
            case MOTIF_GPP:
                artifactMotif = ArtifactMotifEnum.GREEN_PURPLE_PURPLE;
                break;
            case MOTIF_PGP:
                artifactMotif = ArtifactMotifEnum.PURPLE_GREEN_PURPLE;
                break;
            case MOTIF_PPG:
                artifactMotif = ArtifactMotifEnum.PURPLE_PURPLE_GREEN;
                break;
        }
    }
}

