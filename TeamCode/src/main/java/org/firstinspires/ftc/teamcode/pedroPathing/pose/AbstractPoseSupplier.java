package org.firstinspires.ftc.teamcode.pedroPathing.pose;

import com.pedropathing.geometry.Pose;

public abstract class AbstractPoseSupplier {

  protected static final double DISTANCE_TO_TRAVEL_DURING_INTAKE = 26.5;
  protected static final double DISTANCE_TO_TRAVEL_DURING_INTAKE_GROUP1 = 18;
  protected static final double CLASSIFIER_Y_POSITION = 66;
  protected static final double CLASSIFIER_OPEN_HEADING = 90;

  //ARTIFACT GROUP SHARED POSITIONS
  protected static final double ARTIFACT_GROUP_4_Y_POSITION = 27;
  protected static final double ARTIFACT_GROUP_4_HEADING = Math.toRadians(270);
  protected static final double ARTIFACT_GROUP_3_Y_POSITION = 35;
  protected static final double ARTIFACT_GROUP_2_Y_POSITION = 58;
  protected static final double ARTIFACT_GROUP_1_Y_POSITION = 82;

  //BLUE SPECIFIC
  protected static final double BLUE_DEFAULT_HEADING = Math.toRadians(180);
  protected static final double BLUE_NEAR_START_X_POSITION = 22;
  protected static final double BLUE_FAR_START_X_POSITION = 54;
  protected static final double BLUE_ARTIFACT_GROUP_START_X_POSITION = 44;
  protected static final double BLUE_ARTIFACT_GROUP_4_X_POSITION = 2;
  protected static final double BLUE_NEAR_LAUNCH_X_POSITION = 53;
  protected static final double BLUE_FAR_LAUNCH_X_POSITION = 52;
  protected static final Pose BLUE_END_POSE = new Pose(48, 126, BLUE_DEFAULT_HEADING);
  protected static final Pose BLUE_CLASSIFIER_GATE_POSE = new Pose(BLUE_ARTIFACT_GROUP_START_X_POSITION, CLASSIFIER_Y_POSITION, Math.toRadians(CLASSIFIER_OPEN_HEADING));
  protected static final Pose BLUE_CLASSIFIER_GATE_OPEN_POSE = new Pose(BLUE_ARTIFACT_GROUP_START_X_POSITION - DISTANCE_TO_TRAVEL_DURING_INTAKE_GROUP1, CLASSIFIER_Y_POSITION, Math.toRadians(CLASSIFIER_OPEN_HEADING));


  //RED SPECIFIC
  protected static final double RED_DEFAULT_HEADING = Math.toRadians(0);
  protected static final double RED_NEAR_START_X_POSITION = 121;
  protected static final double RED_FAR_START_X_POSITION = 86;
  protected static final double RED_ARTIFACT_GROUP_START_X_POSITION = 101;
  protected static final double RED_ARTIFACT_GROUP_4_X_POSITION = 142;
  protected static final double RED_NEAR_LAUNCH_X_POSITION = 82;
  protected static final double RED_FAR_LAUNCH_X_POSITION = 90;
  protected static final Pose RED_END_POSE = new Pose(96, 126, RED_DEFAULT_HEADING);
  protected static final Pose RED_CLASSIFIER_GATE_POSE = new Pose(RED_ARTIFACT_GROUP_START_X_POSITION, CLASSIFIER_Y_POSITION, Math.toRadians(CLASSIFIER_OPEN_HEADING));
  protected static final Pose RED_CLASSIFIER_GATE_OPEN_POSE = new Pose(RED_ARTIFACT_GROUP_START_X_POSITION + DISTANCE_TO_TRAVEL_DURING_INTAKE_GROUP1, CLASSIFIER_Y_POSITION, Math.toRadians(CLASSIFIER_OPEN_HEADING));


  //NEAR SPECIFIC
  protected static final double NEAR_LAUNCH_Y_POSITION = 88;
  protected static final double NEAR_START_Y_POSITION = 121;

  //FAR SPECIFIC
  protected static final double FAR_LAUNCH_Y_POSITION = 13;
  protected static final double FAR_START_Y_POSITION = 7;

  public abstract Pose getStartPose();
  public abstract Pose getEndPose();
  public abstract Pose getLaunchPose();
  public abstract Pose getArtifactGroup1StartIntakePose();
  public abstract Pose getArtifactGroup1EndIntakePose();
  public abstract Pose getArtifactGroup2StartIntakePose();
  public abstract Pose getArtifactGroup2EndIntakePose();
  public abstract Pose getArtifactGroup3StartIntakePose();
  public abstract Pose getArtifactGroup3EndIntakePose();
  public abstract Pose getArtifactGroup4StartIntakePose();
  public abstract Pose getArtifactGroup4EndIntakePose();
  public abstract Pose getClassifierGateStartOpenPose();
  public abstract Pose getClassifierGateEndOpenPose();
}
