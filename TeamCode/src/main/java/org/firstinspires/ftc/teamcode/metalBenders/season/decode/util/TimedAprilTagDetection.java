package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TimedAprilTagDetection {
    private final AprilTagDetection detection;
    private final long startTime;

    public TimedAprilTagDetection(AprilTagDetection detection) {
        this.detection = detection;
        this.startTime = System.currentTimeMillis();
    }

    public AprilTagDetection getDetection() {
        return detection;
    }

    public long getAgeInMillis() {
        return System.currentTimeMillis() - startTime;
    }
}
