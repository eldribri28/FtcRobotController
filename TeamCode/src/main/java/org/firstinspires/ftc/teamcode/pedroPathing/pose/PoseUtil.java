package org.firstinspires.ftc.teamcode.pedroPathing.pose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PoseUtil {

  public static PathChain buildLinearPathChainBetweenTwoPoses(Follower follower, Pose startPose, Pose endPose) {
    return follower
        .pathBuilder()
        .addPath(new BezierLine(startPose, endPose))
        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
        .setVelocityConstraint(0)
        .build();
  }

  public static PathChain buildLinearPathChainOutAndBack(Follower follower, Pose startAndEndPose, Pose midpoint) {
    return follower
        .pathBuilder()
        .addPath(new BezierLine(startAndEndPose, midpoint))
        .setLinearHeadingInterpolation(startAndEndPose.getHeading(), midpoint.getHeading())
        .addPath(new BezierLine(midpoint, startAndEndPose))
        .setLinearHeadingInterpolation(midpoint.getHeading(), startAndEndPose.getHeading())
        .setVelocityConstraint(0)
        .build();
  }
}
