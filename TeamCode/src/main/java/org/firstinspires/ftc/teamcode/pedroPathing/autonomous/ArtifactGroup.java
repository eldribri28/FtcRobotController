package org.firstinspires.ftc.teamcode.pedroPathing.autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.autonomous.AutonomousStateEnum.DRIVE_FROM_NEAR_LAUNCH_TO_FAR_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.autonomous.AutonomousStateEnum.DRIVE_FROM_NEAR_LAUNCH_TO_MIDDLE_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.autonomous.AutonomousStateEnum.DRIVE_FROM_NEAR_LAUNCH_TO_NEAR_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.autonomous.AutonomousStateEnum.SHOOT_PRELOAD;

public enum ArtifactGroup {
  PRELOAD(SHOOT_PRELOAD),
  NEAR(DRIVE_FROM_NEAR_LAUNCH_TO_NEAR_ARTIFACT_GROUP),
  MIDDLE(DRIVE_FROM_NEAR_LAUNCH_TO_MIDDLE_ARTIFACT_GROUP),
  FAR(DRIVE_FROM_NEAR_LAUNCH_TO_FAR_ARTIFACT_GROUP),
  LOADING_ZONE(SHOOT_PRELOAD);

  private AutonomousStateEnum startingState;

  ArtifactGroup(AutonomousStateEnum startingState) {
    this.startingState = startingState;
  }

  private AutonomousStateEnum getStartingState() {
    return startingState;
  }
}
