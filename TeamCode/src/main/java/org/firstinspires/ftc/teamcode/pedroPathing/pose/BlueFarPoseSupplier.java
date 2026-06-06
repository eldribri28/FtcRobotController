package org.firstinspires.ftc.teamcode.pedroPathing.pose;

import com.pedropathing.geometry.Pose;

public class BlueFarPoseSupplier extends AbstractPoseSupplier {

  @Override
  public Pose getStartPose() {
    return new Pose(
        BLUE_FAR_START_X_POSITION,
        FAR_START_Y_POSITION,
        BLUE_DEFAULT_HEADING);
  }

  @Override
  public Pose getLaunchPose() {
    return new Pose(
        BLUE_FAR_LAUNCH_X_POSITION,
        FAR_LAUNCH_Y_POSITION,
        BLUE_DEFAULT_HEADING);
  }

  @Override
  public Pose getArtifactGroup3StartIntakePose() {
    return new Pose(
        BLUE_ARTIFACT_GROUP_START_X_POSITION,
        ARTIFACT_GROUP_3_Y_POSITION,
        BLUE_DEFAULT_HEADING);
  }

  @Override
  public Pose getArtifactGroup3EndIntakePose() {
    return new Pose(
        BLUE_ARTIFACT_GROUP_START_X_POSITION - DISTANCE_TO_TRAVEL_DURING_INTAKE,
        ARTIFACT_GROUP_3_Y_POSITION,
        BLUE_DEFAULT_HEADING);
  }

  @Override
  public Pose getArtifactGroup2StartIntakePose() {
    return new Pose(
        BLUE_ARTIFACT_GROUP_START_X_POSITION,
        ARTIFACT_GROUP_2_Y_POSITION,
        BLUE_DEFAULT_HEADING
    );
  }

  @Override
  public Pose getArtifactGroup2EndIntakePose() {
    return new Pose(
        BLUE_ARTIFACT_GROUP_START_X_POSITION - DISTANCE_TO_TRAVEL_DURING_INTAKE,
        ARTIFACT_GROUP_2_Y_POSITION,
        BLUE_DEFAULT_HEADING);
  }

  @Override
  public Pose getArtifactGroup1StartIntakePose() {
    return new Pose(
        BLUE_ARTIFACT_GROUP_START_X_POSITION,
        ARTIFACT_GROUP_1_Y_POSITION,
        BLUE_DEFAULT_HEADING
    );
  }

  @Override
  public Pose getArtifactGroup1EndIntakePose() {
    return new Pose(
        BLUE_ARTIFACT_GROUP_START_X_POSITION - DISTANCE_TO_TRAVEL_DURING_INTAKE_GROUP1,
        ARTIFACT_GROUP_1_Y_POSITION,
        BLUE_DEFAULT_HEADING);
  }

  @Override
  public Pose getArtifactGroup4StartIntakePose() {
    return new Pose(
        BLUE_ARTIFACT_GROUP_4_X_POSITION,
        ARTIFACT_GROUP_4_Y_POSITION,
        ARTIFACT_GROUP_4_HEADING);
  }

  @Override
  public Pose getArtifactGroup4EndIntakePose() {
    return new Pose(
        BLUE_ARTIFACT_GROUP_4_X_POSITION,
        ARTIFACT_GROUP_4_Y_POSITION - DISTANCE_TO_TRAVEL_DURING_INTAKE,
        ARTIFACT_GROUP_4_HEADING);
  }

  @Override
  public Pose getClassifierGateStartOpenPose() {
    return BLUE_CLASSIFIER_GATE_POSE;
  }

  @Override
  public Pose getClassifierGateEndOpenPose() {
    return BLUE_CLASSIFIER_GATE_OPEN_POSE;
  }

  @Override
  public Pose getEndPose() {
    return BLUE_END_POSE;
  }
}
