package org.firstinspires.ftc.teamcode.pedroPathing.pose;

import com.pedropathing.geometry.Pose;

public class BlueFarPoseSupplier extends AbstractPoseSupplier{
  @Override
  public Pose getStartPose() {
    return null;
  }

  @Override
  public Pose getLaunchPose() {
    return new Pose(
            BLUE_FAR_LAUNCH_X_POSITION,
            FAR_LAUNCH_Y_POSITION,
            BLUE_DEFAULT_HEADING);
  }

  @Override
  public Pose getNearArtifactGroupPose() {
    return new Pose(
            BLUE_ARTIFACT_GROUP_4_X_POSITION,
            ARTIFACT_GROUP_3_Y_POSITION,
            BLUE_DEFAULT_HEADING);
  }

  @Override
  public Pose getNearArtifactEndIntakePose() {
    return null;
  }

  @Override
  public Pose getMiddleArtifactGroupPose() {
    return null;
  }

  @Override
  public Pose getMiddleArtifactEndIntakePose() {
    return null;
  }

  @Override
  public Pose getFarArtifactGroupPose() {
    return null;
  }

  @Override
  public Pose getFarArtifactEndIntakePose() {
    return null;
  }

  @Override
  public Pose getLoadingZoneArtifactGroupPose() {
    return null;
  }

  @Override
  public Pose getLoadingZoneArtifactEndIntakePose() {
    return null;
  }

  @Override
  public Pose getEndPose() {
    return null;
  }
}
