package org.firstinspires.ftc.teamcode.metalBenders.ignore.systems;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.types.SystemPriorityEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AutoTargetingLaunchSystem extends AbstractSystem {

    private final AprilTagEnum targetAprilTag;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private boolean isMotifSet = false;
    private final PIDController bearingPid = new PIDController(0.05, 0.005, 0.05);


    public AutoTargetingLaunchSystem(AprilTagEnum targetAprilTag) {
        this.targetAprilTag = targetAprilTag;
        this.aprilTagProcessor = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(getHardwareManager().getTurretCam());
        builder.addProcessor(aprilTagProcessor);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(640, 480));
        this.visionPortal = builder.build();
    }

    @Override
    public SystemPriorityEnum getSystemPriority() {
        return SystemPriorityEnum.PRIORITY_2;
    }
    @Override
    protected void start() {
        setExposureAndGain();
    }

    @Override
    protected void process() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        addTelemetry("# AprilTags Detected", currentDetections.size());
        if (currentDetections.isEmpty()) {
            getHardwareManager().getTurretMotor().setPower(0);
        } else {
            for (AprilTagDetection detection : currentDetections) {
                if(detection != null && detection.metadata != null) {
                    processDetection(detection);
                }
            }
        }
    }

    @Override
    protected void shutdown() {
        visionPortal.close();
    }

    private void setExposureAndGain() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if(exposureControl != null && gainControl != null)  {
            if(exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(5, TimeUnit.MILLISECONDS);
            gainControl.setGain(75);
        }
    }

    private void processDetection(AprilTagDetection detection) {
        AprilTagEnum aprilTagEnum = AprilTagEnum.findById(detection.id);
        if (aprilTagEnum != null) {
            if (targetAprilTag == aprilTagEnum) {
                processTargetDetection(detection);
            } else {
                if(!isMotifSet) {
                    setArtifactMotif(aprilTagEnum);
                }
            }
        }
    }

    private void setArtifactMotif(AprilTagEnum aprilTagEnum) {
        ArtifactMotifEnum artifactMotif;
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
            default:
                artifactMotif = null;
                break;
        }
        if (artifactMotif != null) {
            getDataManager().setArtifactMotif(artifactMotif);
            addTelemetry("Artifact Motif", artifactMotif.name());
            isMotifSet = true;
        }
    }

    private void processTargetDetection(AprilTagDetection targetDetection) {
        getHardwareManager().getTurretMotor().setPower(bearingPid.calculate(1, targetDetection.ftcPose.bearing));
        double targetDistance = targetDetection.ftcPose.range / 39.37;
        double flywheelRPM = (getHardwareManager().getLauncherMotor().getVelocity()/28.0) * 60.0;
        addTelemetry("flywheel RPM", flywheelRPM);

        LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult(flywheelRPM, targetDistance);
        if(launchResult != null) {
            setLaunchAngle(launchResult.getLaunchAngle());
            double targetRPM = Math.round(((targetDistance / 1.670) * 800) + 1900);
            getHardwareManager().getLauncherMotor().setVelocity((targetRPM / 60.0 ) * 28.0);
            addTelemetry("target RPM", flywheelRPM);

            if(getHardwareManager().getGamepad1().right_trigger > 0) {
                getHardwareManager().getLauncherMotor().setVelocity(((targetRPM / 60.0 ) * 28.0) + (( 300.0 / 60.0 ) * 28.0));
                if (Math.abs(((getHardwareManager().getLauncherMotor().getVelocity() / 28.0) * 60.0) - targetRPM) < 150.0 ) {
                    if (getDataManager().getLauncherBallColor() != ArtifactColorEnum.NONE) {
                        autoLaunchBall();
                    }
                    autoIntakeBall();
                }
            } else {
                getHardwareManager().getLauncherMotor().setVelocity(0);
            }
        }
    }

    private void autoLaunchBall() {
        getHardwareManager().getLaunchServo().setPosition(0);
        sleep(100);
        getHardwareManager().getLaunchServo().setPosition(0.7);
        sleep(100);
    }

    private void autoIntakeBall() {
        getHardwareManager().getIntakeMotor().setPower(1);
        sleep(25);
        getHardwareManager().getIntakeMotor().setPower(0);
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((62.0-launchAngle)/35.0));
        getHardwareManager().getAngleServo().setPosition(positionValue);
        addTelemetry("launchAngle", launchAngle);
    }
}
