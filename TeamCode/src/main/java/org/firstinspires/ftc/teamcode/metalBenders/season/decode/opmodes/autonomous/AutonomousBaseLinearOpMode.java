package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.autonomous;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public abstract class AutonomousBaseLinearOpMode extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
    private HardwareManager hardwareManager;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

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
            if(!isGainAndExposureSet) {
                setExposureAndGain();
                isGainAndExposureSet = true;
            }
            //TODO fill in meat
            telemetry.update();
        }
        visionPortal.close();
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        this.aprilTagProcessor = new AprilTagProcessor.Builder().build();
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
        if(exposureControl != null && gainControl != null)  {
            if(exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(5, TimeUnit.MILLISECONDS);
            gainControl.setGain(75);
        }
    }
}
