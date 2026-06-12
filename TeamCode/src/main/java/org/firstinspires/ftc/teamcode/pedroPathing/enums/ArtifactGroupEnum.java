package org.firstinspires.ftc.teamcode.pedroPathing.enums;

import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.DRIVE_FROM_LAUNCH_TO_END;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_3;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_4;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_2;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_1;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.SHOOT_PRELOAD;

public enum ArtifactGroupEnum {
  PRELOAD(SHOOT_PRELOAD),
  ARTIFACT_GROUP_1(DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_1),
  ARTIFACT_GROUP_2(DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_2),
  ARTIFACT_GROUP_3(DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_3),
  ARTIFACT_GROUP_4(DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_4),
  NONE(DRIVE_FROM_LAUNCH_TO_END);

  private final AutonomousStateEnum startingState;

  ArtifactGroupEnum(AutonomousStateEnum startingState) {
    this.startingState = startingState;
  }

  public AutonomousStateEnum getStartingState() {
    return startingState;
  }
}