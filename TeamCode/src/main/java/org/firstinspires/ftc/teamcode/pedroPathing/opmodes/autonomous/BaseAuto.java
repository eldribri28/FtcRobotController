package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.APRIL_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.NO_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.VIABLE_LAUNCH_SOLUTION;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum.NEAR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_NO_POWER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_POWER_IN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCHER_MOTOR_IDLE_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_GATE_OPEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCH_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MIN_LAUNCH_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TARGET_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_TARGET_CLOSE_RATE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CHASSIS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.ShotCalculator.calculateLeadAngle;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.ShotCalculator.updateTargetDiff;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.NONE;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.PRELOAD_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.pose.PoseUtil.buildLinearPathChainBetweenTwoPoses;
import static org.firstinspires.ftc.teamcode.pedroPathing.pose.PoseUtil.buildLinearPathChainOutAndBack;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.IndicatorLedEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.AbstractPoseSupplier;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Iterator;
import java.util.List;

public abstract class BaseAuto extends LinearOpMode {
    private static final double ABORT_TIME_LIMIT = 2;
    private static final double AUTO_TIME_DURATION = 30;
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double flywheelRPM = 0;
    private double targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
    private double launchVelocity = 0;
    private double launchTOF = 0;
    private double launchLeadAngle = 0;
    private double targetBearing = 0;
    private double targetYaw = 0;
    private boolean launchSolution = false;
    private HardwareManager hardwareManager;
    private AprilTagEngine aprilTagEngine;
    private Thread aprilTagEngineThread;
    private Follower follower;
    private final Iterator<ArtifactGroupEnum> artifactGroupIterator =
        getArtifactGroupExecutionOrder().iterator();
    private ArtifactGroupEnum currentArtifactGroup;
    private AutonomousStateEnum currentState;
    private int initialTurretPosition;

    //PATH CHAINS
    private PathChain startToLaunch;
    private PathChain launchToNearArtifactGroup;
    private PathChain intakeNearArtifactGroup;
    private PathChain nearArtifactGroupToLaunch;
    private PathChain launchToMiddleArtifactGroup;
    private PathChain intakeMiddleArtifactGroup;
    private PathChain middleArtifactGroupToLaunch;
    private PathChain launchToFarArtifactGroup;
    private PathChain intakeFarArtifactGroup;
    private PathChain farArtifactGroupToLaunch;
    private PathChain launchToLoadingZoneArtifactGroup;
    private PathChain intakeLoadingZoneArtifactGroup;
    private PathChain loadingZoneArtifactGroupToLaunch;
    private PathChain launchToEnd;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();
            hardwareManager.getAngleServo().setPosition(0);
            aprilTagEngineThread.start();
            waitForStart();
            resetRuntime();
            hardwareManager.postStartInitialization();
            while (opModeIsActive() && currentState != COMPLETE) {
                updateState();
                autoLaunch();
                updateTelemetry();
            }
        } finally {
            if(aprilTagEngineThread != null) {
                aprilTagEngineThread.interrupt();
                aprilTagEngine.teardown();
            }
            updateTelemetry();
        }
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        aprilTagEngineThread = new Thread(aprilTagEngine);
        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.RED.getLedValue());
        hardwareManager.getIntakeServo().setPosition(INTAKE_DOWN);
        initialTurretPosition = hardwareManager.getTurretMotor().getCurrentPosition();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        setInitialState();
    }

    private void setInitialState() {
        updateToNextArtifactGroup();
        // If we are starting with shooting preload, and we are starting at the near launch zone,
        // we must drive to shoot first
        if(currentArtifactGroup == PRELOAD_ARTIFACT_GROUP && getStartPosition() == NEAR) {
            currentState = DRIVE_FROM_START_TO_LAUNCH;
        }
    }

    private void updateToNextArtifactGroup() {
        if(artifactGroupIterator.hasNext()) {
            currentArtifactGroup = artifactGroupIterator.next();
        } else {
            currentArtifactGroup = NONE;
        }
        currentState = currentArtifactGroup.getStartingState();
    }

    private void updateState() {
        if(shouldAbort()) {
            follower.breakFollowing();
            currentArtifactGroup = NONE;
            currentState = ABORT;
        }
        follower.update();
        if(!follower.isBusy()) {
            switch(currentArtifactGroup) {
                case PRELOAD_ARTIFACT_GROUP:
                    updatePreloadStates();
                    break;
                case NEAR_ARTIFACT_GROUP:
                    updateNearArtifactGroupStates();
                    break;
                case MIDDLE_ARTIFACT_GROUP:
                    updateMiddleArtifactGroupState();
                    break;
                case FAR_ARTIFACT_GROUP:
                    updateFarArtifactGroupState();
                    break;
                case LOADING_ZONE_ARTIFACT_GROUP:
                    updateLoadingZoneArtifactGroupState();
                    break;
                case NONE:
                    updateNoneArtifactGroupState();
                    break;
            }
        }
    }

    private boolean shouldAbort() {
        return AUTO_TIME_DURATION - getRuntime() < ABORT_TIME_LIMIT
            && currentState != ABORT
            && currentState != DRIVE_FROM_LAUNCH_TO_END;
    }

    private void updatePreloadStates() {
        switch(currentState) {
            //PRELOAD STATES
            case DRIVE_FROM_START_TO_LAUNCH:
                follower.followPath(startToLaunch, true);
                currentState = SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateNearArtifactGroupStates() {
        switch (currentState) {
            //NEAR ARTIFACT GROUP
            case DRIVE_FROM_LAUNCH_TO_NEAR_ARTIFACT_GROUP:
                follower.followPath(launchToNearArtifactGroup, true);
                currentState = INTAKE_NEAR_ARTIFACT_GROUP;
                break;
            case INTAKE_NEAR_ARTIFACT_GROUP:
                startIntake();
                follower.followPath(intakeNearArtifactGroup);
                currentState = DRIVE_FROM_NEAR_ARTIFACT_GROUP_TO_LAUNCH;
                break;
            case DRIVE_FROM_NEAR_ARTIFACT_GROUP_TO_LAUNCH:
                stopIntake();
                follower.followPath(nearArtifactGroupToLaunch);
                currentState = SHOOT_NEAR_ARTIFACT_GROUP;
                break;
            case SHOOT_NEAR_ARTIFACT_GROUP:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateMiddleArtifactGroupState() {
        switch (currentState) {
            //MIDDLE ARTIFACT GROUP
            case DRIVE_FROM_LAUNCH_TO_MIDDLE_ARTIFACT_GROUP:
                follower.followPath(launchToMiddleArtifactGroup, true);
                currentState = INTAKE_MIDDLE_ARTIFACT_GROUP;
                break;
            case INTAKE_MIDDLE_ARTIFACT_GROUP:
                startIntake();
                follower.followPath(intakeMiddleArtifactGroup);
                currentState = DRIVE_FROM_MIDDLE_ARTIFACT_GROUP_TO_LAUNCH;
                break;
            case DRIVE_FROM_MIDDLE_ARTIFACT_GROUP_TO_LAUNCH:
                stopIntake();
                follower.followPath(middleArtifactGroupToLaunch);
                currentState = SHOOT_MIDDLE_ARTIFACT_GROUP;
                break;
            case SHOOT_MIDDLE_ARTIFACT_GROUP:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateFarArtifactGroupState() {
        switch (currentState) {
            //FAR ARTIFACT_GROUP
            case DRIVE_FROM_LAUNCH_TO_FAR_ARTIFACT_GROUP:
                follower.followPath(launchToFarArtifactGroup, true);
                currentState = INTAKE_FAR_ARTIFACT_GROUP;
                break;
            case INTAKE_FAR_ARTIFACT_GROUP:
                startIntake();
                follower.followPath(intakeFarArtifactGroup);
                currentState = DRIVE_FROM_FAR_ARTIFACT_GROUP_TO_LAUNCH;
                break;
            case DRIVE_FROM_FAR_ARTIFACT_GROUP_TO_LAUNCH:
                stopIntake();
                follower.followPath(farArtifactGroupToLaunch);
                currentState = SHOOT_FAR_ARTIFACT_GROUP;
                break;
            case SHOOT_FAR_ARTIFACT_GROUP:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateLoadingZoneArtifactGroupState() {
        switch (currentState) {
            //LOADING ZONE ARTIFACT_GROUP
            case DRIVE_FROM_LAUNCH_TO_LOADING_ZONE_ARTIFACT_GROUP:
                follower.followPath(launchToLoadingZoneArtifactGroup, true);
                currentState = INTAKE_LOADING_ZONE_ARTIFACT_GROUP;
                break;
            case INTAKE_LOADING_ZONE_ARTIFACT_GROUP:
                startIntake();
                follower.followPath(intakeLoadingZoneArtifactGroup);
                currentState = DRIVE_FROM_LOADING_ZONE_ARTIFACT_GROUP_TO_LAUNCH;
                break;
            case DRIVE_FROM_LOADING_ZONE_ARTIFACT_GROUP_TO_LAUNCH:
                stopIntake();
                follower.followPath(loadingZoneArtifactGroupToLaunch);
                currentState = SHOOT_LOADING_ZONE_ARTIFACT_GROUP;
                break;
            case SHOOT_LOADING_ZONE_ARTIFACT_GROUP:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateNoneArtifactGroupState() {
        switch (currentState) {
            case DRIVE_FROM_LAUNCH_TO_END:
                follower.followPath(launchToEnd);
                currentState = ENDING_STATE;
                break;
            case ABORT:
                PathChain pathChain = buildLinearPathChainBetweenTwoPoses(
                    follower, follower.getPose(), getPoseSupplier().getEndPose());
                follower.followPath(pathChain);
                currentState = ENDING_STATE;
                break;
            case ENDING_STATE:
                currentState = COMPLETE;
                break;
        }
    }

    private void shootAndUpdateToNextArtifactGroup() {
        if (readyToShoot()) {
            sleep(1000);
            autoLaunchArtifact();
            sleep(1000);
            stopLaunchArtifact();
            updateToNextArtifactGroup();
        }
    }

    private void buildPaths() {
        AbstractPoseSupplier poseSupplier = getPoseSupplier();

        //Handles Move to Launch Position from Starting Position (for Near Start)
        startToLaunch = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getStartPose(), poseSupplier.getLaunchPose());

        // Handle Move to Near Artifacts, Intake them, Return to Launch Position
        launchToNearArtifactGroup = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getStartPose(), poseSupplier.getNearArtifactGroupPose());
        intakeNearArtifactGroup = buildLinearPathChainOutAndBack(
                follower, poseSupplier.getNearArtifactGroupPose(), poseSupplier.getNearArtifactEndIntakePose());
        nearArtifactGroupToLaunch = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getNearArtifactGroupPose(), poseSupplier.getLaunchPose());

        // Handle Move to Middle Artifacts, Intake them, Return to Launch Position
        launchToMiddleArtifactGroup = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getLaunchPose(), poseSupplier.getMiddleArtifactGroupPose());
        intakeMiddleArtifactGroup = buildLinearPathChainOutAndBack(
                follower, poseSupplier.getMiddleArtifactGroupPose(), poseSupplier.getMiddleArtifactEndIntakePose());
        middleArtifactGroupToLaunch = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getMiddleArtifactGroupPose(), poseSupplier.getLaunchPose());

        // Handle Move to Far Artifacts, Intake them, Return to Launch Position
        launchToFarArtifactGroup = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getLaunchPose(), poseSupplier.getFarArtifactGroupPose());
        intakeFarArtifactGroup = buildLinearPathChainOutAndBack(
            follower, poseSupplier.getFarArtifactGroupPose(), poseSupplier.getFarArtifactEndIntakePose());
        farArtifactGroupToLaunch = buildLinearPathChainBetweenTwoPoses(
                follower, poseSupplier.getFarArtifactGroupPose(), poseSupplier.getLaunchPose());

        // Handle Move to Loading Zone Artifacts, Intake them, Return to Launch Position
        launchToLoadingZoneArtifactGroup = buildLinearPathChainBetweenTwoPoses(
                follower, poseSupplier.getLaunchPose(), poseSupplier.getLoadingZoneArtifactGroupPose());
        intakeLoadingZoneArtifactGroup = buildLinearPathChainOutAndBack(
                follower, poseSupplier.getLoadingZoneArtifactGroupPose(), poseSupplier.getLoadingZoneArtifactEndIntakePose());
        loadingZoneArtifactGroupToLaunch = buildLinearPathChainBetweenTwoPoses(
                follower, poseSupplier.getLoadingZoneArtifactGroupPose(), poseSupplier.getLaunchPose());

        // Move from Launch Position to Park Positon
        launchToEnd = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getLaunchPose(), poseSupplier.getEndPose());

        // Set Starting Pose
        follower.setStartingPose(poseSupplier.getStartPose());
    }

    private void startIntake() {
        hardwareManager.getIntakeMotor().setPower(INTAKE_POWER_IN);
    }

    private void stopIntake() {
        hardwareManager.getIntakeMotor().setPower(INTAKE_NO_POWER);
    }

    private void autoLaunch() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if(timedDetection != null && timedDetection.getDetection() != null) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
                setLedStates(APRIL_TAG_DETECTED);

                targetDistance = targetDetection.ftcPose.range;
                targetBearing = targetDetection.ftcPose.bearing;
                targetYaw = targetDetection.ftcPose.yaw;
                flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                telemetry.addData("flywheel RPM", flywheelRPM);

                updateTargetDiff(targetYaw, targetDistance);

                LaunchCalculator.LaunchResult launchResult = LaunchCalculator.getLaunchData(LAUNCH_HEIGHT, TARGET_HEIGHT, targetDistance, flywheelRPM, ROBOT_TARGET_CLOSE_RATE);
                launchVelocity = launchResult.getLaunchVelocity();
                launchAngle = launchResult.getFlyWheelAngle();
                launchTOF = launchResult.getTOF();
                if (launchTOF > 0) {
                    launchLeadAngle = calculateLeadAngle(launchTOF);
                } else {
                    launchLeadAngle = 0;
                }

                ROBOT_FIELD_X = targetDetection.robotPose.getPosition().x;
                ROBOT_FIELD_Y = targetDetection.robotPose.getPosition().y;

                if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(targetBearing) && targetDetection.id == getTargetAprilTag().getId()) {
                    telemetry.addData("Turret Angle", (targetBearing));
                    telemetry.addData("Target ID", targetDetection.id);

                    double turretError = targetBearing - launchLeadAngle;
                    double setPower = turretBearingPid.calculate(0,turretError) * 0.6;
                    telemetry.addData("Turret Power", setPower);
                    hardwareManager.getTurretMotor().setPower(setPower);

                    if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
                        TURRET_CHASSIS_OFFSET = getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition());
                        double chassisFieldHeading = (targetDetection.robotPose.getOrientation().getYaw() - TURRET_CHASSIS_OFFSET);
                        telemetry.addData("AprilTag Turret Field Heading (deg)", targetDetection.robotPose.getOrientation().getYaw());
                        telemetry.addData("Calculated Chassis Field Heading (deg)", chassisFieldHeading);
                    }
                } else {
                    hardwareManager.getTurretMotor().setPower(0);
                    handleNoTagDetected();
                }

                if (launchVelocity > 0 || launchAngle > 0) {
                    launchSolution = true;
                    hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.YELLOW.getLedValue());
                    setLaunchAngle(launchAngle);
                    targetRPM = Math.round(launchResult.getFlywheelRpm());
                } else {
                    launchSolution = false;
                    hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.ORANGE.getLedValue());
                }
                telemetry.addData("target RPM", targetRPM);
            } else {
                handleNoTagDetected();
            }
        } else {
            handleNoTagDetected();
        }
        hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
    }

    private void setLedStates(LedStateEnum ledStateEnum) {
        if(ledStateEnum == APRIL_TAG_DETECTED) {
            hardwareManager.getRedLed().on();
            hardwareManager.getGreenLed().off();
        } else if (ledStateEnum == VIABLE_LAUNCH_SOLUTION) {
            hardwareManager.getRedLed().off();
            hardwareManager.getGreenLed().on();
        } else {
            hardwareManager.getRedLed().off();
            hardwareManager.getGreenLed().off();
        }
    }

    private boolean canRotateTurret(double input) {
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()
            || input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()) {
            return false;
        } else {
            return true;
        }
    }

    private void handleNoTagDetected() {
        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.RED.getLedValue());
        targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
        hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_CLOSE);
        hardwareManager.getTurretMotor().setTargetPosition(initialTurretPosition);
        setLedStates(NO_TAG_DETECTED);
    }

    public boolean readyToShoot() {
        return isLaunchMotorVelocityWithinThreshold()
            && isTurretAngleWithinThreshold(aprilTagEngine.getTimedTargetDetection().getDetection())
            && launchSolution;
    }

    private boolean isTurretAngleWithinThreshold(AprilTagDetection detection) {
        double bearing = Math.abs(detection.ftcPose.bearing);
        if(targetDistance > 2.0) {
            return bearing <= 0.5;
        }
        return bearing <= 1.0;
    }

    private boolean isLaunchMotorVelocityWithinThreshold() {
        double currentRPM = ((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0);
        return Math.abs(targetRPM - currentRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
    }

    private void setLaunchAngle(double launchAngle) {
        if (launchAngle != 0) {
            double positionValue = Range.clip(((launchAngle - MIN_LAUNCH_ANGLE) * (0.65/(MAX_LAUNCH_ANGLE-MIN_LAUNCH_ANGLE)) - 0), 0.0, 0.7);
            hardwareManager.getAngleServo().setPosition(positionValue);
            this.launchAngle = launchAngle;
        }
    }

    private void autoLaunchArtifact() {
        hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_OPEN);
        hardwareManager.getIntakeMotor().setPower(INTAKE_POWER_IN);
    }

    private void stopLaunchArtifact() {
        hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_CLOSE);
        hardwareManager.getIntakeMotor().setPower(INTAKE_NO_POWER);
    }

    private void updateTelemetry() {
        updateRuntime();
        telemetry.addData("Target april tag", getTargetAprilTag().name());
        telemetry.addData("Start position", getStartPosition().name());
        telemetry.addData("Artifact group execution order", getArtifactGroupExecutionOrder().toString());
        telemetry.addData("Current artifact group", currentArtifactGroup.name());
        telemetry.addData("Current state", currentState.name());
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
            "Runtime",
            "%02.0f:%02.0f",
            totalSeconds / 60,
            totalSeconds % 60);
        telemetry.addData(
            "Time remaining",
            "%02.0f:%02.0f",
            (30 - totalSeconds) / 60,
            (30 - totalSeconds) % 60);
    }

    abstract AprilTagEnum getTargetAprilTag();
    abstract StartPositionEnum getStartPosition();
    abstract List<ArtifactGroupEnum> getArtifactGroupExecutionOrder();
    abstract AbstractPoseSupplier getPoseSupplier();
}