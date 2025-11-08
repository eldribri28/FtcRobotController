package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.GREEN_PURPLE_PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.PURPLE_GREEN_PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.PURPLE_PURPLE_GREEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.UNKNOWN;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_GAIN_SET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.CAMERA_GAIN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.CAMERA_EXPOSURE;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum;
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

public class AprilTagEngine implements Runnable {
    private final HardwareManager hardwareManager;
    private final AprilTagEnum targetAprilTag;
    private final Map<String, String> telemetry = new ConcurrentHashMap<>();
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private ArtifactMotifEnum artifactMotif = UNKNOWN;
    private final ReentrantReadWriteLock artifactMotifLock = new ReentrantReadWriteLock();
    private TimedAprilTagDetection targetDetection = null;
    private final ReentrantReadWriteLock targetDetectionLock = new ReentrantReadWriteLock();

    public AprilTagEngine(HardwareManager hardwareManager, AprilTagEnum targetAprilTag) {
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
                telemetry.put("AprilTagEngine exception",
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
            if (artifactMotif == UNKNOWN) {
                setArtifactMotif(aprilTagEnum);
            }
            if (detection.metadata != null
                    && detection.ftcPose != null
                    && targetAprilTag == aprilTagEnum) {
                setTargetDetection(detection);
            }
        }
    }

    private void setArtifactMotif(AprilTagEnum aprilTagEnum) {
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

    public TimedAprilTagDetection getTimedTargetDetection() {
        targetDetectionLock.readLock().lock();
        try {
            return targetDetection;
        } finally {
            targetDetectionLock.readLock().unlock();
        }
    }

    private void setTargetDetection(AprilTagDetection detection) {
        targetDetectionLock.writeLock().lock();
        try {
            this.targetDetection = new TimedAprilTagDetection(detection);
        } finally {
            targetDetectionLock.writeLock().unlock();
        }
    }

    public Map<String, String> getTelemetry() {
        return new HashMap<>(telemetry);
    }
}
