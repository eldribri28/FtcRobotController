package org.firstinspires.ftc.teamcode.pedroPathing.pose;

import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.ARTIFACT_GROUP_1;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.ARTIFACT_GROUP_2;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.ARTIFACT_GROUP_3;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.ARTIFACT_GROUP_4;
import static org.firstinspires.ftc.teamcode.pedroPathing.pose.PoseUtil.buildLinearPathChainBetweenPoses;
import static org.firstinspires.ftc.teamcode.pedroPathing.pose.PoseUtil.buildIntakePathChainWithOpenGate;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum;

import java.util.List;

public class PathSupplier {

    //PATH CHAINS
    private PathChain startToLaunch;
    private PathChain launchToArtifactGroup1;
    private PathChain intakeArtifactGroup1;
    private PathChain driveFromArtifactGroup1ToLaunch;
    private PathChain driveFromLaunchToArtifactGroup2;
    private PathChain intakeArtifactGroup2;
    private PathChain driveFromArtifactGroup2ToLaunch;
    private PathChain driveFromLaunchToArtifactGroup3;
    private PathChain intakeArtifactGroup3;
    private PathChain driveFromArtifactGroup3ToLaunch;
    private PathChain driveFromLaunchToArtifactGroup4;
    private PathChain intakeArtifactGroup4;
    private PathChain driveFromArtifactGroup4ToLaunch;
    private PathChain driveFromLaunchToEnd;

    public PathSupplier(
            Follower follower,
            AbstractPoseSupplier poseSupplier,
            List<ArtifactGroupEnum> groupsToEmptyClassifierAfterIntake) {
        buildPaths(follower, poseSupplier, groupsToEmptyClassifierAfterIntake);
    }

    private void buildPaths(
            Follower follower,
            AbstractPoseSupplier poseSupplier,
            List<ArtifactGroupEnum> groupsToEmptyClassifierAfterIntake) {
        //Handles Move to Launch Position from Starting Position (for Near Start)
        startToLaunch = buildLinearPathChainBetweenPoses(
                follower,
                poseSupplier.getStartPose(),
                poseSupplier.getLaunchPose());

        // Move from Launch Position to Park Positon
        driveFromLaunchToEnd = buildLinearPathChainBetweenPoses(
                follower,
                poseSupplier.getLaunchPose(),
                poseSupplier.getEndPose());

        buildArtifactGroup1Paths(follower, poseSupplier, groupsToEmptyClassifierAfterIntake);
        buildArtifactGroup2Paths(follower, poseSupplier, groupsToEmptyClassifierAfterIntake);
        buildArtifactGroup3Paths(follower, poseSupplier, groupsToEmptyClassifierAfterIntake);
        buildArtifactGroup4Paths(follower, poseSupplier, groupsToEmptyClassifierAfterIntake);
    }

    private void buildArtifactGroup1Paths(
            Follower follower,
            AbstractPoseSupplier poseSupplier,
            List<ArtifactGroupEnum> groupsToEmptyClassifierAfterIntake) {
        // Handle Move to Artifact group 1, Intake them, Return to Launch Position
        launchToArtifactGroup1 = buildLinearPathChainBetweenPoses(
                follower,
                poseSupplier.getStartPose(),
                poseSupplier.getArtifactGroup1StartIntakePose());
        if(groupsToEmptyClassifierAfterIntake.contains(ARTIFACT_GROUP_1)) {
            intakeArtifactGroup1 = buildIntakePathChainWithOpenGate(
                    follower,
                    poseSupplier.getArtifactGroup1StartIntakePose(),
                    poseSupplier.getArtifactGroup1EndIntakePose(),
                    poseSupplier.getClassifierGateStartOpenPose(),
                    poseSupplier.getClassifierGateEndOpenPose());
            driveFromArtifactGroup1ToLaunch = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getClassifierGateStartOpenPose(),
                    poseSupplier.getLaunchPose());
        } else {
            intakeArtifactGroup1 = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getArtifactGroup1StartIntakePose(),
                    poseSupplier.getArtifactGroup1EndIntakePose(),
                    poseSupplier.getArtifactGroup1StartIntakePose());
            driveFromArtifactGroup1ToLaunch = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getArtifactGroup1StartIntakePose(),
                    poseSupplier.getLaunchPose());
        }
    }

    private void buildArtifactGroup2Paths(
            Follower follower,
            AbstractPoseSupplier poseSupplier,
            List<ArtifactGroupEnum> groupsToEmptyClassifierAfterIntake) {
        // Handle Move to Artifact group 2, Intake them, Return to Launch Position
        driveFromLaunchToArtifactGroup2 = buildLinearPathChainBetweenPoses(
                follower,
                poseSupplier.getLaunchPose(),
                poseSupplier.getArtifactGroup2StartIntakePose());
        if(groupsToEmptyClassifierAfterIntake.contains(ARTIFACT_GROUP_2)) {
            intakeArtifactGroup2 = buildIntakePathChainWithOpenGate(
                    follower,
                    poseSupplier.getArtifactGroup2StartIntakePose(),
                    poseSupplier.getArtifactGroup2EndIntakePose(),
                    poseSupplier.getClassifierGateStartOpenPose(),
                    poseSupplier.getClassifierGateEndOpenPose());
            driveFromArtifactGroup2ToLaunch = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getClassifierGateStartOpenPose(),
                    poseSupplier.getLaunchPose());
        } else {
            intakeArtifactGroup2 = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getArtifactGroup2StartIntakePose(),
                    poseSupplier.getArtifactGroup2EndIntakePose(),
                    poseSupplier.getArtifactGroup2StartIntakePose());
            driveFromArtifactGroup2ToLaunch = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getArtifactGroup2StartIntakePose(),
                    poseSupplier.getLaunchPose());
        }
    }

    private void buildArtifactGroup3Paths(
            Follower follower,
            AbstractPoseSupplier poseSupplier,
            List<ArtifactGroupEnum> groupsToEmptyClassifierAfterIntake) {
        // Handle Move to Artifact group 3, Intake them, Return to Launch Position
        driveFromLaunchToArtifactGroup3 = buildLinearPathChainBetweenPoses(
                follower,
                poseSupplier.getLaunchPose(),
                poseSupplier.getArtifactGroup3StartIntakePose());
        if(groupsToEmptyClassifierAfterIntake.contains(ARTIFACT_GROUP_3)) {
            intakeArtifactGroup3 = buildIntakePathChainWithOpenGate(
                    follower,
                    poseSupplier.getArtifactGroup3StartIntakePose(),
                    poseSupplier.getArtifactGroup3EndIntakePose(),
                    poseSupplier.getClassifierGateStartOpenPose(),
                    poseSupplier.getClassifierGateEndOpenPose());
            driveFromArtifactGroup3ToLaunch = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getClassifierGateStartOpenPose(),
                    poseSupplier.getLaunchPose());
        } else {
            intakeArtifactGroup3 = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getArtifactGroup3StartIntakePose(),
                    poseSupplier.getArtifactGroup3EndIntakePose(),
                    poseSupplier.getArtifactGroup3StartIntakePose());
            driveFromArtifactGroup3ToLaunch = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getArtifactGroup3StartIntakePose(),
                    poseSupplier.getLaunchPose());
        }
    }

    private void buildArtifactGroup4Paths(
            Follower follower,
            AbstractPoseSupplier poseSupplier,
            List<ArtifactGroupEnum> groupsToEmptyClassifierAfterIntake) {
        // Handle Move to Artifact group 4, Intake them, Return to Launch Position
        driveFromLaunchToArtifactGroup4 = buildLinearPathChainBetweenPoses(
                follower,
                poseSupplier.getLaunchPose(),
                poseSupplier.getArtifactGroup4StartIntakePose());

        if(groupsToEmptyClassifierAfterIntake.contains(ARTIFACT_GROUP_4)) {
            intakeArtifactGroup4 = buildIntakePathChainWithOpenGate(
                    follower,
                    poseSupplier.getArtifactGroup4StartIntakePose(),
                    poseSupplier.getArtifactGroup4EndIntakePose(),
                    poseSupplier.getClassifierGateStartOpenPose(),
                    poseSupplier.getClassifierGateEndOpenPose());
            driveFromArtifactGroup4ToLaunch = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getClassifierGateStartOpenPose(),
                    poseSupplier.getLaunchPose());
        } else {
            intakeArtifactGroup4 = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getArtifactGroup4StartIntakePose(),
                    poseSupplier.getArtifactGroup4EndIntakePose(),
                    poseSupplier.getArtifactGroup4StartIntakePose());
            driveFromArtifactGroup4ToLaunch = buildLinearPathChainBetweenPoses(
                    follower,
                    poseSupplier.getArtifactGroup4StartIntakePose(),
                    poseSupplier.getLaunchPose());
        }
    }

    public PathChain getStartToLaunch() {
        return startToLaunch;
    }

    public PathChain getLaunchToArtifactGroup1() {
        return launchToArtifactGroup1;
    }

    public PathChain getIntakeArtifactGroup1() {
        return intakeArtifactGroup1;
    }

    public PathChain getDriveFromArtifactGroup1ToLaunch() {
        return driveFromArtifactGroup1ToLaunch;
    }

    public PathChain getDriveFromLaunchToArtifactGroup2() {
        return driveFromLaunchToArtifactGroup2;
    }

    public PathChain getIntakeArtifactGroup2() {
        return intakeArtifactGroup2;
    }

    public PathChain getDriveFromArtifactGroup2ToLaunch() {
        return driveFromArtifactGroup2ToLaunch;
    }

    public PathChain getDriveFromLaunchToArtifactGroup3() {
        return driveFromLaunchToArtifactGroup3;
    }

    public PathChain getIntakeArtifactGroup3() {
        return intakeArtifactGroup3;
    }

    public PathChain getDriveFromArtifactGroup3ToLaunch() {
        return driveFromArtifactGroup3ToLaunch;
    }

    public PathChain getDriveFromLaunchToArtifactGroup4() {
        return driveFromLaunchToArtifactGroup4;
    }

    public PathChain getIntakeArtifactGroup4() {
        return intakeArtifactGroup4;
    }

    public PathChain getDriveFromArtifactGroup4ToLaunch() {
        return driveFromArtifactGroup4ToLaunch;
    }

    public PathChain getDriveFromLaunchToEnd() {
        return driveFromLaunchToEnd;
    }
}