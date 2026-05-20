package org.firstinspires.ftc.teamcode.pedroPathing.autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;

public class AutonomousPoseManager {

    private final StartPositionEnum startPositionEnum;
    private final AprilTagEnum targetAprilTag;

    public AutonomousPoseManager(StartPositionEnum startPositionEnum, AprilTagEnum targetAprilTag) {
        this.startPositionEnum = startPositionEnum;
        this.targetAprilTag = targetAprilTag;
    }

    public Pose getStartPose() {
        return new Pose(54,8,Math.toRadians(180));
    }

    public Pose getNearLaunchPose() {
        return new Pose(54,20,Math.toRadians(180));
    }

    public Pose getFarLaunchPose() {
        return new Pose(54,18,Math.toRadians(180));
    }

    public Pose getNearArtifactGroupPose() {
        return new Pose(41,35,Math.toRadians(180));
    }

    public Pose getNearArtifactEndIntakePose() {
        return new Pose(17,35,Math.toRadians(180));
    }

    public Pose getMiddleArtifactGroupPose() {
        return new Pose(41,58,Math.toRadians(180));
    }

    public Pose getMiddleArtifactEndIntakePose() {
        return new Pose(17,58,Math.toRadians(180));
    }

    public Pose getFarArtifactGroupPose() {
        return new Pose(41,82,Math.toRadians(180));
    }

    public Pose getFarArtifactEndIntakePose() {
        return new Pose(17,82,Math.toRadians(180));
    }

    public Pose getEndPose() {
        return new Pose(41,70,Math.toRadians(180));
    }
}
