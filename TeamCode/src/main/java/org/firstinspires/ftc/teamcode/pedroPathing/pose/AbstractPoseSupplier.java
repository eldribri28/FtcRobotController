package org.firstinspires.ftc.teamcode.pedroPathing.pose;

import com.pedropathing.geometry.Pose;

public abstract class AbstractPoseSupplier {

  protected static final double DISTANCE_TO_TRAVEL_DURING_INTAKE = 26;

  //ARTIFACT GROUP SHARED POSITIONS
  protected static final double ARTIFACT_GROUP_4_Y_POSITION = 27;
  protected static final double ARTIFACT_GROUP_4_HEADING = Math.toRadians(270);
  protected static final double ARTIFACT_GROUP_3_Y_POSITION = 35;
  protected static final double ARTIFACT_GROUP_2_Y_POSITION = 58;
  protected static final double ARTIFACT_GROUP_1_Y_POSITION = 82;

  //BLUE SPECIFIC
  protected static final double BLUE_DEFAULT_HEADING = Math.toRadians(180);
  protected static final double BLUE_NEAR_START_HEADING = Math.toRadians(144);
  protected static final double BLUE_NEAR_START_X_POSITION = 20;
  protected static final double BLUE_FAR_START_X_POSITION = 54;
  protected static final double BLUE_ARTIFACT_GROUP_START_X_POSITION = 42;
  protected static final double BLUE_ARTIFACT_GROUP_4_X_POSITION = 10;
  protected static final double BLUE_NEAR_LAUNCH_X_POSITION = 62;
  protected static final double BLUE_FAR_LAUNCH_X_POSITION = 52;
  protected static final Pose BLUE_END_POSE = new Pose(60, 90, Math.toRadians(180));

  //RED SPECIFIC
  protected static final double RED_DEFAULT_HEADING = Math.toRadians(0);
  protected static final double RED_NEAR_START_HEADING = Math.toRadians(36);
  protected static final double RED_NEAR_START_X_POSITION = 121;
  protected static final double RED_FAR_START_X_POSITION = 86;
  protected static final double RED_ARTIFACT_GROUP_START_X_POSITION = 101;
  protected static final double RED_ARTIFACT_GROUP_4_X_POSITION = 133;
  protected static final double RED_NEAR_LAUNCH_X_POSITION = 82;
  protected static final double RED_FAR_LAUNCH_X_POSITION = 90;
  protected static final Pose RED_END_POSE = new Pose(4, 90, Math.toRadians(0));

  //NEAR SPECIFIC
  protected static final double NEAR_LAUNCH_Y_POSITION = 84;
  protected static final double NEAR_START_Y_POSITION = 120;

  //FAR SPECIFIC
  protected static final double FAR_LAUNCH_Y_POSITION = 10;
  protected static final double FAR_START_Y_POSITION = 8;

  public abstract Pose getStartPose();
  public abstract Pose getLaunchPose();
  public abstract Pose getNearArtifactGroupPose();
  public abstract Pose getNearArtifactEndIntakePose();
  public abstract Pose getMiddleArtifactGroupPose();
  public abstract Pose getMiddleArtifactEndIntakePose();
  public abstract Pose getFarArtifactGroupPose();
  public abstract Pose getFarArtifactEndIntakePose();
  public abstract Pose getLoadingZoneArtifactGroupPose();
  public abstract Pose getLoadingZoneArtifactEndIntakePose();
  public abstract Pose getEndPose();
}
