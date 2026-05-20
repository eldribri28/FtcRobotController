package org.firstinspires.ftc.teamcode.pedroPathing.pose;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;

public abstract class AbstractPoseSupplier {

  protected static final double DISTANCE_TO_TRAVEL_DURING_INTAKE = 20;
  protected static final double ARTIFACT_GROUP_4_HEADING = Math.toRadians(270);

  //ARTIFACT GROUP X POSITIONS
  protected static final double ARTIFACT_GROUP_4_Y_POSITION = 27;
  protected static final double ARTIFACT_GROUP_3_Y_POSITION = 35;
  protected static final double ARTIFACT_GROUP_2_Y_POSITION = 58;
  protected static final double ARTIFACT_GROUP_1_Y_POSITION = 82;


  //BLUE SPECIFIC
  protected static final double BLUE_DEFAULT_HEADING = Math.toRadians(180);
  protected static final double BLUE_NEAR_START_HEADING = Math.toRadians(144);
  protected static final double BLUE_ARTIFACT_GROUP_START_X_POSITION = 41;
  protected static final double BLUE_ARTIFACT_GROUP_4_X_POSITION = 8;


  //RED SPECIFIC
  protected static final double RED_DEFAULT_HEADING = Math.toRadians(0);
  protected static final double RED_NEAR_START_HEADING = Math.toRadians(36);
  protected static final double RED_ARTIFACT_GROUP_START_X_POSITION = 101;
  protected static final double RED_ARTIFACT_GROUP_4_X_POSITION = 133;


  //NEAR SPECIFIC
  protected static final double NEAR_LAUNCH_Y_POSITION = 112;
  protected static final double NEAR_START_Y_POSITION = 122;


  //FAR SPECIFIC
  protected static final double FAR_LAUNCH_Y_POSITION = 13;
  protected static final double FAR_START_Y_POSITION = 8;

  public abstract Pose getStartPose();
  public abstract Pose getNearLaunchPose();
  public abstract Pose getFarLaunchPose();
  public abstract Pose getNearArtifactGroupPose();
  public abstract Pose getNearArtifactEndIntakePose();
  public abstract Pose getMiddleArtifactGroupPose();
  public abstract Pose getMiddleArtifactEndIntakePose();
  public abstract Pose getFarArtifactGroupPose();
  public abstract Pose getFarArtifactEndIntakePose();
  public abstract Pose getEndPose();

  public static AbstractPoseSupplier getPoseSupplier(StartPositionEnum startPositionEnum, AprilTagEnum targetAprilTag) {
    if (startPositionEnum == StartPositionEnum.NEAR && targetAprilTag == AprilTagEnum.BLUE_TARGET) {
      return new BlueNearPoseSupplier();
    } else if (startPositionEnum == StartPositionEnum.FAR && targetAprilTag == AprilTagEnum.BLUE_TARGET) {
      return new BlueFarPoseSupplier();
    } else if (startPositionEnum == StartPositionEnum.NEAR && targetAprilTag == AprilTagEnum.RED_TARGET) {
      return new RedNearPoseSupplier();
    } else if (startPositionEnum == StartPositionEnum.FAR && targetAprilTag == AprilTagEnum.RED_TARGET) {
      return new RedFarPoseSupplier();
    } else {
      throw new IllegalArgumentException("Invalid start position or target April Tag");
    }
  }
}
