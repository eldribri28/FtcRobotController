package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

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

    /*
    Blue Near:
        getStartPose: Pose(23,122,Math.toRadians(135))
        getNearLaunchPose: Pose(62,84,Math.toRadians(180))
        getFarLaunchPose: Pose(54,12,Math.toRadians(180))
        getNearArtifactGroupPose: Pose(41,82,Math.toRadians(180))
        getNearArtifactEndIntakePose: Pose(20,82,Math.toRadians(180))
        getMiddleArtifactGroupPose: Pose(41,58,Math.toRadians(180))
        getMiddleArtifactEndIntakePose: Pose(17,58,Math.toRadians(180))
        getFarArtifactGroupPose: Pose(41,35,Math.toRadians(180))
        getFarArtifactEndIntakePose: Pose(17,35,Math.toRadians(180))
        getEndPose: Pose(60,90,Math.toRadians(180))

    Blue Far:
        getStartPose: Pose(54,8,Math.toRadians(180))
        getNearLaunchPose: Pose(54,12,Math.toRadians(180))
        getFarLaunchPose: Pose(54,90,Math.toRadians(180))
        getNearArtifactGroupPose: Pose(41,35,Math.toRadians(180))
        getNearArtifactEndIntakePose: Pose(17,35,Math.toRadians(180))
        getMiddleArtifactGroupPose: Pose(41,58,Math.toRadians(180))
        getMiddleArtifactEndIntakePose: Pose(17,58,Math.toRadians(180))
        getFarArtifactGroupPose: Pose(41,82,Math.toRadians(180))
        getFarArtifactEndIntakePose: Pose(20,82,Math.toRadians(180))
        getEndPose: Pose(60,90,Math.toRadians(180))

    Red Near:
        getStartPose: Pose(121,122,Math.toRadians(45))
        getNearLaunchPose: Pose(82,84,Math.toRadians(0))
        getFarLaunchPose: Pose(90,12,Math.toRadians(0))
        getNearArtifactGroupPose: Pose(103,82,Math.toRadians(0))
        getNearArtifactEndIntakePose: Pose(124,82,Math.toRadians(0))
        getMiddleArtifactGroupPose: Pose(103,58,Math.toRadians(0))
        getMiddleArtifactEndIntakePose: Pose(127,58,Math.toRadians(0))
        getFarArtifactGroupPose: Pose(103,35,Math.toRadians(0))
        getFarArtifactEndIntakePose: Pose(127,35,Math.toRadians(0))
        getEndPose: Pose(84,90,Math.toRadians(0))

    Red Far:
        getStartPose: Pose(90,8,Math.toRadians(0))
        getNearLaunchPose: Pose(90,12,Math.toRadians(0))
        getFarLaunchPose: Pose(90,90,Math.toRadians(0))
        getNearArtifactGroupPose: Pose(103,35,Math.toRadians(0))
        getNearArtifactEndIntakePose: Pose(127,35,Math.toRadians(0))
        getMiddleArtifactGroupPose: Pose(103,58,Math.toRadians(0))
        getMiddleArtifactEndIntakePose: Pose(127,58,Math.toRadians(0))
        getFarArtifactGroupPose: Pose(103,82,Math.toRadians(0))
        getFarArtifactEndIntakePose: Pose(124,82,Math.toRadians(0))
        getEndPose: Pose(84,90,Math.toRadians(0))

     */





    public Pose getStartPose() {
        return new Pose(54,8,Math.toRadians(180));
    }

    public Pose getNearLaunchPose() {
        return new Pose(54,12,Math.toRadians(180));
    }

    public Pose getFarLaunchPose() {
        return new Pose(54,90,Math.toRadians(180));
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
        return new Pose(20,82,Math.toRadians(180));
    }

    public Pose getEndPose() {
        return new Pose(60,90,Math.toRadians(180));
    }
}
