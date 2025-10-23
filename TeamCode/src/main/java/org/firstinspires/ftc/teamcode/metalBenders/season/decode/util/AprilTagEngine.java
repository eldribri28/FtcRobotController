package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum.BLUE_TARGET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum.RED_TARGET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.GREEN_PURPLE_PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum.UNKNOWN;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class AprilTagEngine implements Runnable {
    private final HardwareManager hardwareManager;
    private final Telemetry telemetry;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private ArtifactMotifEnum artifactMotif = UNKNOWN;
    private final ReentrantReadWriteLock artifactMotifLock = new ReentrantReadWriteLock();
    private AprilTagDetection blueTargetDetection = null;
    private final ReentrantReadWriteLock blueTargetDetectionLock = new ReentrantReadWriteLock();
    private AprilTagDetection redTargetDetection = null;
    private final ReentrantReadWriteLock redTargetDetectionLock = new ReentrantReadWriteLock();

    public AprilTagEngine(HardwareManager hardwareManager, Telemetry telemetry) {
        this.hardwareManager = hardwareManager;
        this.telemetry = telemetry;
        initialize();
    }

    private void initialize() {
        Position cameraPosition = new Position(DistanceUnit.METER, 0,  -0.20, 0.250, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        aprilTagProcessorBuilder.setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.364794005);
        aprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
        this.aprilTagProcessor = aprilTagProcessorBuilder.build();
        aprilTagProcessor.setDecimation(4);
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareManager.getTurretCam());
        builder.addProcessor(aprilTagProcessor);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(640, 480));
        this.visionPortal = builder.build();
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()
                && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e){}
        }
        setExposureAndGain();
        while (!Thread.currentThread().isInterrupted()) {
            try {
                detectAndProcessAprilTags();
            } catch (Exception e) {
                telemetry.addData("AprilTagEngine exception",
                        e.getClass().getSimpleName() + " - " + e.getMessage());
            }
        }
        visionPortal.close();
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

    private void detectAndProcessAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection != null) {
                processDetection(detection);
            }
        }
    }

    private void processDetection(AprilTagDetection detection) {
        if(detection != null) {
            AprilTagEnum aprilTagEnum = AprilTagEnum.findById(detection.id);
            if (aprilTagEnum != null) {
                if (artifactMotif == UNKNOWN) {
                    setArtifactMotif(aprilTagEnum);
                }
                if (detection.metadata != null) {
                    if(BLUE_TARGET == aprilTagEnum) {
                        setBlueTargetDetection(detection);
                    } else if (RED_TARGET == aprilTagEnum) {
                        setRedTargetDetection(detection);
                    }
                }
            }
        }
    }

    private void setArtifactMotif(AprilTagEnum aprilTagEnum) {
        switch (aprilTagEnum) {
            case MOTIF_GPP:
                setArtifactMotif(GREEN_PURPLE_PURPLE);
                break;
            case MOTIF_PGP:
                setArtifactMotif(ArtifactMotifEnum.PURPLE_GREEN_PURPLE);
                break;
            case MOTIF_PPG:
                setArtifactMotif(ArtifactMotifEnum.PURPLE_PURPLE_GREEN);
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

    public AprilTagDetection getBlueTargetDetection() {
        blueTargetDetectionLock.readLock().lock();
        try {
            return blueTargetDetection;
        } finally {
            blueTargetDetectionLock.readLock().unlock();
        }
    }

    private void setBlueTargetDetection(AprilTagDetection detection) {
        blueTargetDetectionLock.writeLock().lock();
        try {
            this.blueTargetDetection = detection;
        } finally {
            blueTargetDetectionLock.writeLock().unlock();
        }
    }

    public AprilTagDetection getRedTargetDetection() {
        redTargetDetectionLock.readLock().lock();
        try {
            return redTargetDetection;
        } finally {
            redTargetDetectionLock.readLock().unlock();
        }
    }

    private void setRedTargetDetection(AprilTagDetection detection) {
        redTargetDetectionLock.writeLock().lock();
        try {
            this.redTargetDetection = detection;
        } finally {
            redTargetDetectionLock.writeLock().unlock();
        }
    }
}
