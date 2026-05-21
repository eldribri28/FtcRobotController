package org.firstinspires.ftc.teamcode.pedroPathing.enums;

import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.DRIVE_FROM_LAUNCH_TO_FAR_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.DRIVE_FROM_LAUNCH_TO_MIDDLE_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.DRIVE_FROM_LAUNCH_TO_NEAR_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.END_STATE;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.SHOOT_PRELOAD;

public enum ArtifactGroupEnum {
  PRELOAD_ARTIFACT_GROUP(SHOOT_PRELOAD),
  NEAR_ARTIFACT_GROUP(DRIVE_FROM_LAUNCH_TO_NEAR_ARTIFACT_GROUP),
  MIDDLE_ARTIFACT_GROUP(DRIVE_FROM_LAUNCH_TO_MIDDLE_ARTIFACT_GROUP),
  FAR_ARTIFACT_GROUP(DRIVE_FROM_LAUNCH_TO_FAR_ARTIFACT_GROUP),
  LOADING_ZONE_ARTIFACT_GROUP(SHOOT_PRELOAD),
  NONE(END_STATE);

  private AutonomousStateEnum startingState;

  ArtifactGroupEnum(AutonomousStateEnum startingState) {
    this.startingState = startingState;
  }

  public AutonomousStateEnum getStartingState() {
    return startingState;
  }
}
