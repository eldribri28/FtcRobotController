package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.SYSTEM_LATENCY;

import android.view.WindowId;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

import java.util.ArrayList;
import java.util.List;

public class localization {

    public static Pose2D aprilTagLatencyCompensatedPose(long frameAcquisitionNanoTime, Pose2D robotPose) {
        Pose2D matchedHistoricalPose = poseToPose2D(getPoseAtTime(frameAcquisitionNanoTime), DistanceUnit.METER, AngleUnit.RADIANS);
        Pose2D poseDelta = deltaPose(matchedHistoricalPose, robotPose);
        // Project the vision measurement forward into the present time
        // Inject corrected coordinates back into the follower, test blending the updated pose with current pose (need to add kalman filter)
        return robotPose; //offsetPose2DByPose2D(robotPose, poseDelta);
    }

    public static Pose2D poseToPose2D(Pose thePose, DistanceUnit distanceUnit, AngleUnit angleUnit) {
        return new Pose2D(distanceUnit, thePose.getX(), thePose.getY(), angleUnit, thePose.getHeading());
    }

    public static Pose2D offsetPose2DByPose2D(Pose2D basePose, Pose2D offsetPose) {
        double headingRad = basePose.getHeading(AngleUnit.RADIANS);
        double localX = offsetPose.getX(DistanceUnit.METER);
        double localY = offsetPose.getY(DistanceUnit.METER);
        double rotatedX = localX * Math.cos(headingRad) - localY * Math.sin(headingRad);
        double rotatedY = localX * Math.sin(headingRad) + localY * Math.cos(headingRad);
        double newX = basePose.getX(DistanceUnit.METER) + rotatedX;
        double newY = basePose.getY(DistanceUnit.METER) + rotatedY;
        double newHeading = basePose.getHeading(AngleUnit.RADIANS) + offsetPose.getHeading(AngleUnit.RADIANS);
        newHeading = AngleUnit.normalizeRadians(newHeading);
        return new Pose2D(DistanceUnit.METER, newX, newY, AngleUnit.RADIANS, newHeading);
    }

    public static Pose2D editPose2DAngle(Pose2D pose, double angle) {
        return new Pose2D(DistanceUnit.METER, pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER), AngleUnit.RADIANS, angle);
    }

    public static double convertTo360Degrees(double angle) {
        if (angle < 0.0) { angle += 360.0; }
        if (angle > 360.0) { angle -= 360.0; }
        return angle;
    }

    public static void updatePedroFromRobotPose(Follower pedroFollower, Pose2D robotPose) {
        Pose ftcToPedro = convertToPedroPose(robotPose, InvertedFTCCoordinates.INSTANCE);
        pedroFollower.setPose(ftcToPedro);
        pedroFollower.update();
    }

    public static Pose2D updateRobotPoseFromPedro(Follower pedroFollower) {
        pedroFollower.update();
        Pose currentPose = pedroFollower.getPose();
        Pose pedroToFtc = currentPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        double robotX = pedroToFtc.getX() * 0.0254;
        double robotY = pedroToFtc.getY() * 0.0254;
        double robotH = pedroToFtc.getHeading();
        addPoseHistory(new Pose(robotX, robotY, robotH)); // Add pose to history for latency compensation
        return new Pose2D(DistanceUnit.METER, robotX, robotY, AngleUnit.RADIANS, robotH);
    }

    public static Pose convertToPedroPose(Pose2D ftcPoseMeters, CoordinateSystem coordinateSystem) {
        double xInches = ftcPoseMeters.getX(DistanceUnit.INCH);
        double yInches = ftcPoseMeters.getY(DistanceUnit.INCH);
        double heading = ftcPoseMeters.getHeading(AngleUnit.RADIANS);
        Pose pedroPoseStandard = new Pose(xInches, yInches, heading, coordinateSystem);
        return pedroPoseStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public static Pose2D getTurretFieldPose(Pose2D robotPose, Pose2D turretOffsetPose, double turretRelativeAngle) {
        Pose2D currentTurretOffsetPose = editPose2DAngle(turretOffsetPose, turretRelativeAngle);
        return offsetPose2DByPose2D(robotPose, currentTurretOffsetPose);
    }

    public static Pose2D cameraFieldPoseToTurretFieldPose(Pose2D cameraPose, Pose2D cameraPoseOffset) {
        return offsetPose2DByPose2D(cameraPose, cameraPoseOffset);
    }

    public static Pose2D turretFieldPoseToRobotFieldPose(Pose2D turretPose, Pose2D turretOffsetPose, double turretRelativeAngle) {
        Pose2D currentTurretOffsetPose = invertPose(editPose2DAngle(turretOffsetPose, turretRelativeAngle));
        return offsetPose2DByPose2D(turretPose, currentTurretOffsetPose);
    }

    public static double calculateDistanceBetweenPose2D(Pose2D basePose, Pose2D goalPose) {
        double deltaX = goalPose.getX(DistanceUnit.METER) - basePose.getX(DistanceUnit.METER);
        double deltaY = goalPose.getY(DistanceUnit.METER) - basePose.getY(DistanceUnit.METER);
        return Math.hypot(deltaX, deltaY);
    }

    public static double calculateAngleBetweenPose2D(Pose2D currentPose, Pose2D targetPose) {
        double deltaX = targetPose.getX(DistanceUnit.METER) - currentPose.getX(DistanceUnit.METER);
        double deltaY = targetPose.getY(DistanceUnit.METER) - currentPose.getY(DistanceUnit.METER);
        return Math.atan2(deltaY, deltaX) - (Math.PI / 2);
    }

    public static double calculateTurretError(Pose2D turretPose, Pose2D targetPose) {
        double turretHeadingRad = turretPose.getHeading(AngleUnit.RADIANS);
        double absoluteGoalRad = convertToTwoPi(calculateAngleBetweenPose2D(turretPose, targetPose));
        return AngleUnit.normalizeRadians(absoluteGoalRad - turretHeadingRad);
    }

    public static double convertToTwoPi(double radians) {
        return (radians % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
    }

    public static Pose2D deltaPose(Pose2D startPose, Pose2D endPose) {
        // 1. Calculate X and Y difference
        double deltaX = endPose.getX(DistanceUnit.METER) - startPose.getX(DistanceUnit.METER);
        double deltaY = endPose.getY(DistanceUnit.METER) - startPose.getY(DistanceUnit.METER);
        double deltaHeading = AngleUnit.normalizeRadians(endPose.getHeading(AngleUnit.RADIANS) - startPose.getHeading(AngleUnit.RADIANS));
        return new Pose2D(DistanceUnit.METER, deltaX, deltaY, AngleUnit.RADIANS, convertToTwoPi(deltaHeading));
    }

    public static Pose2D invertPose(Pose2D pose) {
        // Extract values in a standard unit (Radians)
        double x = -pose.getX(DistanceUnit.METER);
        double y = -pose.getY(DistanceUnit.METER);
        double headingRad = pose.getHeading(AngleUnit.RADIANS);

        // Invert the rotation component
        double invHeadingRad = -headingRad;

        // Return the new inverted Pose2D
        return new Pose2D(DistanceUnit.METER, x, y, AngleUnit.RADIANS, invHeadingRad);
    }

    public static double velocityToTarget(Pose2D robotPose, Follower follower, AprilTagEnum aprilTag) {
        Vector robotVelocity = follower.getVelocity();
        double targetPoseX;
        double targetPoseY;
        if (aprilTag == AprilTagEnum.BLUE_TARGET) {
            targetPoseX = BLUE_GOAL_POSE.getX(DistanceUnit.METER);
            targetPoseY = BLUE_GOAL_POSE.getY(DistanceUnit.METER);
        } else {
            targetPoseX = RED_GOAL_POSE.getX(DistanceUnit.METER);
            targetPoseY = RED_GOAL_POSE.getY(DistanceUnit.METER);
        }
        double deltaX = targetPoseX - robotPose.getX(DistanceUnit.METER);
        double deltaY = targetPoseY - robotPose.getY(DistanceUnit.METER);
        double distanceToTarget = Math.hypot(deltaX, deltaY);
        double velocityTowardTarget = 0.0;
        if (distanceToTarget > 0.001) {
            double unitX = deltaX / distanceToTarget;
            double unitY = deltaY / distanceToTarget;
            double velocityComponentXInMeters = robotVelocity.getXComponent() * 0.0245;
            double velocityComponentYInMeters = robotVelocity.getYComponent() * 0.0245;
            velocityTowardTarget = (velocityComponentXInMeters * unitX) + (velocityComponentYInMeters * unitY);
        }
        return velocityTowardTarget;
    }

    public static double robotAngularRateToTarget(Pose2D robotPose, Follower follower, AprilTagEnum aprilTag) {
        double velocityMag = follower.getVelocity().getMagnitude() * 0.0254; // Meters per second
        double travelDirectionRad = follower.getVelocity().getTheta(); // Direction of travel
        double robotPoseX = robotPose.getX(DistanceUnit.METER);
        double robotPoseY = robotPose.getY(DistanceUnit.METER);
        double velocityX = velocityMag * Math.cos(travelDirectionRad);
        double velocityY = velocityMag * Math.sin(travelDirectionRad);
        double targetPoseX;
        double targetPoseY;
        if (aprilTag == AprilTagEnum.BLUE_TARGET) {
            targetPoseX = BLUE_GOAL_POSE.getX(DistanceUnit.METER);
            targetPoseY = BLUE_GOAL_POSE.getY(DistanceUnit.METER);
        } else {
            targetPoseX = RED_GOAL_POSE.getX(DistanceUnit.METER);
            targetPoseY = RED_GOAL_POSE.getY(DistanceUnit.METER);
        }
        double deltaX = targetPoseX - robotPoseX;
        double deltaY = targetPoseY - robotPoseY;
        double distanceSquared = (deltaX * deltaX) + (deltaY * deltaY);
        double radius = Math.sqrt(distanceSquared);
        if (radius > 0.001) {
            return ((deltaX * velocityY) - (deltaY * velocityX)) / distanceSquared;
        } else {
            return 0;
        }
    }

    public static Pose2D predictFuturePose(Pose2D currentPose, Follower follower, double timeOfFlight) {
        double totalPredictionTime = SYSTEM_LATENCY + timeOfFlight;
        double velocityMag = follower.getVelocity().getMagnitude() * 0.0254; // Meters per second
        double travelDirectionRad = follower.getVelocity().getTheta(); // Direction of travel
        double angularVelocityRad = follower.getAngularVelocity(); // Radians per second
        double velocityX = velocityMag * Math.cos(travelDirectionRad);
        double velocityY = velocityMag * Math.sin(travelDirectionRad);
        double predictedX = currentPose.getX(DistanceUnit.METER) + (velocityX * totalPredictionTime);
        double predictedY = currentPose.getY(DistanceUnit.METER) + (velocityY * totalPredictionTime);
        double predictedHeadingRad = currentPose.getHeading(AngleUnit.RADIANS) + (angularVelocityRad * totalPredictionTime);
        return new Pose2D(DistanceUnit.METER, predictedX, predictedY, AngleUnit.RADIANS, predictedHeadingRad);
    }

    // historical heading data
    public static class PoseRecord {
        long timestamp;
        Pose currentPose;

        PoseRecord(long timestamp, Pose currentPose) {
            this.timestamp = timestamp;
            this.currentPose = currentPose;
        }
    }

    public static final List<PoseRecord> poseHistory = new ArrayList<>();


    public static void addPoseHistory(Pose currentPose) {
        Pose poseInMeters = new Pose(DistanceUnit.INCH.toMeters(currentPose.getX()), DistanceUnit.INCH.toMeters(currentPose.getY()), currentPose.getHeading());
        poseHistory.add(new PoseRecord(System.nanoTime(), poseInMeters));
        // Limit the history to 1 second (1,000,000,000 nanoseconds) to save memory
        if (poseHistory.size() > 100) {
            poseHistory.remove(0);
        }
    }

    public static Pose getPoseAtTime(long targetTime) {
        if (poseHistory.isEmpty()) {
            return new Pose(0, 0, 0);
        }

        PoseRecord closestRecord = poseHistory.get(0);
        long minDifference = Math.abs(closestRecord.timestamp - targetTime);

        for (PoseRecord record : poseHistory) {
            long diff = Math.abs(record.timestamp - targetTime);
            if (diff < minDifference) {
                minDifference = diff;
                closestRecord = record;
            }
        }

        return closestRecord.currentPose;
    }

    public static void clearPoseHistory() {
        poseHistory.clear();
    }

}
