package org.firstinspires.ftc.teamcode.pedroPathing.pose;

import com.pedropathing.geometry.Pose;

public class RedFarPoseSupplier extends AbstractPoseSupplier{

  @Override
  public Pose getStartPose() {
    return new Pose(
        RED_FAR_START_X_POSITION,
        FAR_START_Y_POSITION,
        RED_DEFAULT_HEADING);
  }

  @Override
  public Pose getLaunchPose() {
    return new Pose(
        RED_FAR_LAUNCH_X_POSITION,
        FAR_LAUNCH_Y_POSITION,
        RED_DEFAULT_HEADING);
  }

  @Override
  public Pose getNearArtifactGroupPose() {
    return new Pose(
        RED_ARTIFACT_GROUP_START_X_POSITION,
        ARTIFACT_GROUP_3_Y_POSITION,
        RED_DEFAULT_HEADING);
  }

  @Override
  public Pose getNearArtifactEndIntakePose() {
    return new Pose(
        RED_ARTIFACT_GROUP_START_X_POSITION + DISTANCE_TO_TRAVEL_DURING_INTAKE,
        ARTIFACT_GROUP_3_Y_POSITION,
        RED_DEFAULT_HEADING);
  }

  @Override
  public Pose getMiddleArtifactGroupPose() {
    return new Pose(
        RED_ARTIFACT_GROUP_START_X_POSITION,
        ARTIFACT_GROUP_2_Y_POSITION,
        RED_DEFAULT_HEADING
    );
  }

  @Override
  public Pose getMiddleArtifactEndIntakePose() {
    return new Pose(
        RED_ARTIFACT_GROUP_START_X_POSITION + DISTANCE_TO_TRAVEL_DURING_INTAKE,
        ARTIFACT_GROUP_2_Y_POSITION,
        RED_DEFAULT_HEADING);
  }

  @Override
  public Pose getFarArtifactGroupPose() {
    return new Pose(
        RED_ARTIFACT_GROUP_START_X_POSITION,
        ARTIFACT_GROUP_1_Y_POSITION,
        RED_DEFAULT_HEADING
    );
  }

  @Override
  public Pose getFarArtifactEndIntakePose() {
    return new Pose(
        RED_ARTIFACT_GROUP_START_X_POSITION + DISTANCE_TO_TRAVEL_DURING_INTAKE_GROUP1,
        ARTIFACT_GROUP_1_Y_POSITION,
        RED_DEFAULT_HEADING);
  }

  @Override
  public Pose getLoadingZoneArtifactGroupPose() {
    return new Pose(
        RED_ARTIFACT_GROUP_4_X_POSITION,
        ARTIFACT_GROUP_4_Y_POSITION,
        ARTIFACT_GROUP_4_HEADING);
  }

  @Override
  public Pose getLoadingZoneArtifactEndIntakePose() {
    return new Pose(
        RED_ARTIFACT_GROUP_4_X_POSITION,
        ARTIFACT_GROUP_4_Y_POSITION + DISTANCE_TO_TRAVEL_DURING_INTAKE,
        ARTIFACT_GROUP_4_HEADING);
  }

  @Override
  public Pose getEndPose() {
    return RED_END_POSE;
  }
}
