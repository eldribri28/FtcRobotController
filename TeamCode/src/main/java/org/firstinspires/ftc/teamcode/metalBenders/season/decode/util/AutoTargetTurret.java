package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.GREEN_PURPLE_PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.PURPLE_GREEN_PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.PURPLE_PURPLE_GREEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.UNKNOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.APRIL_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.NO_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.READY_TO_SHOOT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.VIABLE_LAUNCH_SOLUTION;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.CAMERA_EXPOSURE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.CAMERA_GAIN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_GAIN_SET;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class AutoTargetTurret implements Runnable {
    private final HardwareManager hardwareManager;
    private final AprilTagEnum targetAprilTag;
    private final Map<String, String> telemetry = new ConcurrentHashMap<>();
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private ArtifactMotifEnum artifactMotif = UNKNOWN;
    private final ReentrantReadWriteLock artifactMotifLock = new ReentrantReadWriteLock();
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private long latestTargetDetectionTime = 0L;
    private AprilTagDetection latestTargetDetection = null;
    private double targetRPM = 0.0;

    public AutoTargetTurret(HardwareManager hardwareManager, AprilTagEnum targetAprilTag) {
        this.hardwareManager = hardwareManager;
        this.targetAprilTag = targetAprilTag;
        initialize();
    }

    private void initialize() {
        Position cameraPosition = new Position(DistanceUnit.METER, 0,  -0.20, 0.250, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        aprilTagProcessorBuilder.setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.364794005);
        aprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
        aprilTagProcessorBuilder.setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES);
        this.aprilTagProcessor = aprilTagProcessorBuilder.build();
        aprilTagProcessor.setDecimation(3);
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareManager.getTurretCam());
        builder.addProcessor(aprilTagProcessor);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(640, 480));
        this.visionPortal = builder.build();
    }

    @Override
    public void run() {
        waitForCameraStreamToStart();
        setExposureAndGain();
        while (!Thread.currentThread().isInterrupted()) {
            try {
                detectAndProcessAprilTags();
            } catch (Exception e) {
                telemetry.put("AutoTargetTurret exception",
                        e.getClass().getSimpleName() + " - " + e.getMessage());
            }
        }
    }

    private void waitForCameraStreamToStart() {
        while (!Thread.currentThread().isInterrupted()
                && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e){}
        }
    }

    public void teardown() {
        visionPortal.close();
    }

    private void setExposureAndGain() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (exposureControl != null && gainControl != null && !CAMERA_GAIN_SET) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e){}
            exposureControl.setExposure(CAMERA_EXPOSURE, TimeUnit.MILLISECONDS);
            gainControl.setGain(CAMERA_GAIN);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e){}
            CAMERA_GAIN_SET = true;
        }
    }

    private void detectAndProcessAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getFreshDetections();
        if(currentDetections != null) {
            telemetry.put("# AprilTags Detected", String.valueOf(currentDetections.size()));
            currentDetections.forEach(this::processDetection);
        }
//        else {
//            stopRotatingTurret();
//            setLedStates(NO_TAG_DETECTED);
//        }
        if(System.currentTimeMillis() - latestTargetDetectionTime > TURRET_AGE_DATA_LIMIT_MILLISECONDS) {
            stopRotatingTurret();
            setLedStates(NO_TAG_DETECTED);
        }
    }

    private void processDetection(AprilTagDetection detection) {
        if(detection != null) {
            if (isValidTargetDetection(detection)) {
                processTargetDetection(detection);
            }
            if (artifactMotif == UNKNOWN) {
                setArtifactMotif(detection);
            }
        }
    }

    private boolean isValidTargetDetection(AprilTagDetection detection) {
        return detection.metadata != null
                && detection.ftcPose != null
                && targetAprilTag.getId() == detection.id;
    }

    private void processTargetDetection(AprilTagDetection detection) {
        latestTargetDetectionTime = System.currentTimeMillis();
        latestTargetDetection = detection;
        setLedStates(APRIL_TAG_DETECTED);
        rotateTurretToTarget();
        setLauncherVelocity();
    }

    private void rotateTurretToTarget() {
        if(canRotateTurret(latestTargetDetection.ftcPose.bearing)) {
            double setPower = turretBearingPid.calculate(0, latestTargetDetection.ftcPose.bearing);
            telemetry.put("Turret Power", String.valueOf(setPower));
            hardwareManager.getTurretMotor().setPower(setPower);
        } else {
            stopRotatingTurret();
        }
    }

    private void setLauncherVelocity() {
        double targetDistance = latestTargetDetection.ftcPose.range;

        double flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
        telemetry.put("flywheel RPM", String.valueOf(flywheelRPM));

        LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult(flywheelRPM, targetDistance);
        if (launchResult != null) {
            setLaunchAngle(launchResult.getLaunchAngle());
            setLedStates(VIABLE_LAUNCH_SOLUTION);
        }

        targetRPM = Math.round(((targetDistance / 1.670) * 920.0) + 1750.0);
        hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
        telemetry.put("target RPM", String.valueOf(targetRPM));
        if(readyToShoot()) {
            setLedStates(READY_TO_SHOOT);
        }
    }

    private void setLedStates(LedStateEnum ledStateEnum) {
        if(ledStateEnum == APRIL_TAG_DETECTED) {
            hardwareManager.getRedLed().on();
            hardwareManager.getGreenLed().off();
        } else if (ledStateEnum == VIABLE_LAUNCH_SOLUTION) {
            hardwareManager.getRedLed().on();
            hardwareManager.getGreenLed().on();
        } else if (ledStateEnum == READY_TO_SHOOT) {
            hardwareManager.getRedLed().off();
            hardwareManager.getGreenLed().on();
        } else {
            hardwareManager.getRedLed().off();
            hardwareManager.getGreenLed().off();
        }
    }

    private void stopRotatingTurret() {
        hardwareManager.getTurretMotor().setPower(0);
    }

    private boolean canRotateTurret(double input) {
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()) {
            return false;
        } else if (input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()){
            return false;
        } else {
            return true;
        }
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((70.0 - launchAngle) / 35.0));
        if(positionValue >= 0 && positionValue <= 0.8) {
            hardwareManager.getAngleServo().setPosition(positionValue);
        }
    }

    private void setArtifactMotif(AprilTagDetection detection) {
        AprilTagEnum aprilTagEnum = AprilTagEnum.findById(detection.id);
        if(aprilTagEnum == null) {
            return;
        }
        switch (aprilTagEnum) {
            case MOTIF_GPP:
                setArtifactMotif(GREEN_PURPLE_PURPLE);
                break;
            case MOTIF_PGP:
                setArtifactMotif(PURPLE_GREEN_PURPLE);
                break;
            case MOTIF_PPG:
                setArtifactMotif(PURPLE_PURPLE_GREEN);
                break;
        }
    }

    public ArtifactMotifEnum getArtifactMotif() {
        artifactMotifLock.readLock().lock();
        try {
            return artifactMotif;
        } finally {
            artifactMotifLock.readLock().unlock();
        }
    }

    private void setArtifactMotif(ArtifactMotifEnum artifactMotif) {
        artifactMotifLock.writeLock().lock();
        try {
            this.artifactMotif = artifactMotif;
        } finally {
            artifactMotifLock.writeLock().unlock();
        }
    }

    public Map<String, String> getTelemetry() {
        return new HashMap<>(telemetry);
    }

    public boolean readyToShoot() {
        synchronized (this) {
            return isLatestDetectionValid()
                    && isTurretAngleWithinThreshold()
                    && isLaunchMotorVelocityWithinThreshold();
        }
    }

    private boolean isLatestDetectionValid() {
        return latestTargetDetection != null
                && (System.currentTimeMillis() - latestTargetDetectionTime <= AGED_DATA_LIMIT_MILLISECONDS);
    }
    private boolean isTurretAngleWithinThreshold() {
        return Math.abs(latestTargetDetection.ftcPose.bearing) <= 3.0;
    }

    private boolean isLaunchMotorVelocityWithinThreshold() {
        //TODO should we have multiple constants for max difference for different distance ranges? i.e. allow for larger difference on near shots vs. far shots.
        return Math.abs(((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0) - targetRPM) <= MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
    }
}
