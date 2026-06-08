package org.firstinspires.ftc.teamcode.pedroPathing.pose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class PoseUtil {
  private static final double TIMEOUT_CONSTRAINT = 50;
  private static final double GLOBAL_DECELERATION = 0.2;

  public static PathChain buildLinearPathChainBetweenPoses(
          Follower follower, Pose startPose, Pose ... otherPoses) {
    return buildPathChainBetweenPoses(follower, startPose, otherPoses);
  }

  private static PathChain buildPathChainBetweenPoses(
          Follower follower, Pose startPose, Pose ... otherPoses) {
    PathBuilder pathBuilder = follower.pathBuilder();
    Pose previousPose = startPose;
    for(Pose pose : otherPoses) {
      if(previousPose != null) {
        pathBuilder.addPath(new BezierLine(previousPose, pose));
        pathBuilder.setLinearHeadingInterpolation(previousPose.getHeading(), pose.getHeading());
      }
      previousPose = pose;
    }
    pathBuilder.setTimeoutConstraint(TIMEOUT_CONSTRAINT).setGlobalDeceleration(GLOBAL_DECELERATION);
    return pathBuilder.build();
  }

  public static PathChain buildIntakePathChainWithOpenGate(
          Follower follower,
          Pose artifactGroupStartIntakePose,
          Pose artifactGroupEndIntakePose,
          Pose classiferStartOpenPose,
          Pose classifierEndOpenPose) {
    return buildLinearPathChainBetweenPoses(
            follower,
            //intake
            artifactGroupStartIntakePose,
            artifactGroupEndIntakePose,
            artifactGroupStartIntakePose,
            //open gate
            classiferStartOpenPose,
            classifierEndOpenPose,
            classiferStartOpenPose);
  }
}
