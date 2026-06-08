package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum.NEAR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_IDLE_POWER;
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
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_H;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_TARGET_CLOSE_RATE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.ShotCalculator.updateTargetDiff;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.pose.PoseUtil.buildLinearPathChainBetweenPoses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.IndicatorLedEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.enums.AutonomousStateEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.AbstractPoseSupplier;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.PathSupplier;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Iterator;
import java.util.List;

public abstract class BaseAuto extends LinearOpMode {

    private static final int MIN_READY_COUNT = 2;
    private static final double ABORT_TIME_LIMIT = 0;
    private static final double AUTO_TIME_DURATION = 30;
    private static final double HEADING_ERROR_THRESHOLD = 0.1;
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private int readyToShootCount = 0;
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double flywheelRPM = 0;
    private double targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
    private double launchVelocity = 0;
    private double launchTOF = 0;
    private double launchLeadAngle = 0;
    private double targetBearing = 0;
    private double targetYaw = 0;
    private double turretError = 0;
    private int initialTurretPos = 0;
    private boolean launchSolution = false;
    private HardwareManager hardwareManager;
    private AprilTagEngine aprilTagEngine;
    private Thread aprilTagEngineThread;
    private Follower follower;
    private PathSupplier pathSupplier;
    private final Iterator<ArtifactGroupEnum> artifactGroupIterator =
        getArtifactGroupExecutionOrder().iterator();
    private ArtifactGroupEnum currentArtifactGroup;
    private AutonomousStateEnum currentState;
    private Double shootTime = null;
    private Double preShotTimestamp = null;
    boolean tagDetected = false;

    private List<ArtifactGroupEnum> getArtifactGroupExecutionOrder() {
        //NEAR
        if(NEAR == getStartPosition()) {
            return List.of(
                    PRELOAD,
                    ARTIFACT_GROUP_1,
                    ARTIFACT_GROUP_2,
                    ARTIFACT_GROUP_3,
                    ARTIFACT_GROUP_4
            );
        //FAR
        } else {
            return List.of(
                    PRELOAD,
                    ARTIFACT_GROUP_3,
                    ARTIFACT_GROUP_2,
                    ARTIFACT_GROUP_1,
                    ARTIFACT_GROUP_4
            );
        }
    }

    private List<ArtifactGroupEnum> getArtifactGroupsToEmptyClassifierAfterIntake() {
        return List.of();
//        return List.of(ARTIFACT_GROUP_1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();
            hardwareManager.getAngleServo().setPosition(0);
            initialTurretPos = hardwareManager.getTurretMotor().getCurrentPosition();
            waitForStart();
            resetRuntime();
            aprilTagEngineThread.start();
            hardwareManager.postStartInitialization();
            idleIntake();
            while (opModeIsActive()) {
                updateState();
//                if(currentState.name().startsWith("SHOOT")) {
                    autoLaunch();
//                }
                updateTelemetry();
            }
            stopAllDriveMotors();
        } finally {
            if(aprilTagEngineThread != null) {
                aprilTagEngineThread.interrupt();
                aprilTagEngine.teardown();
            }
        }
    }

    private void stopAllDriveMotors() {
        //stop all drive motors to eliminate drift after time expires or auto session is completed
        hardwareManager.getLeftFrontMotor().setPower(0);
        hardwareManager.getRightFrontMotor().setPower(0);
        hardwareManager.getLeftRearMotor().setPower(0);
        hardwareManager.getRightRearMotor().setPower(0);
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        aprilTagEngineThread = new Thread(aprilTagEngine);
        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.RED.getLedValue());
        hardwareManager.getIntakeServo().setPosition(INTAKE_DOWN);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getPoseSupplier().getStartPose());
        pathSupplier = new PathSupplier(
                follower, getPoseSupplier(), getArtifactGroupsToEmptyClassifierAfterIntake());
        setInitialState();
    }

    private void setInitialState() {
        updateToNextArtifactGroup();
        // If we are starting with shooting preload, and we are starting at the near launch zone,
        // we must drive to shoot first
        if(currentArtifactGroup == PRELOAD && getStartPosition() == NEAR) {
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
        evaluateAndSetAbortState();
        follower.update();
        if(canMoveToNextState()) {
            switch(currentArtifactGroup) {
                case PRELOAD:
                    updatePreloadStates();
                    break;
                case ARTIFACT_GROUP_1:
                    updateArtifactGroup1States();
                    break;
                case ARTIFACT_GROUP_2:
                    updateArtifactGroup2State();
                    break;
                case ARTIFACT_GROUP_3:
                    updateArtifactGroup3State();
                    break;
                case ARTIFACT_GROUP_4:
                    updateArtifactGroup4State();
                    break;
                case NONE:
                    updateNoneArtifactGroupState();
                    break;
            }
        }
    }

    private boolean canMoveToNextState() {
        return !follower.isBusy() || (follower.atParametricEnd() && follower.getHeadingError() < HEADING_ERROR_THRESHOLD);
    }

    private void evaluateAndSetAbortState() {
        if(ABORT_TIME_LIMIT > 0
            && AUTO_TIME_DURATION - getRuntime() < ABORT_TIME_LIMIT
            && currentState != ABORT
            && currentState != DRIVE_FROM_LAUNCH_TO_END) {
            follower.breakFollowing();
            currentArtifactGroup = NONE;
            currentState = ABORT;
        }
    }

    private void updatePreloadStates() {
        switch(currentState) {
            //PRELOAD STATES
            case DRIVE_FROM_START_TO_LAUNCH:
                follower.followPath(pathSupplier.getStartToLaunch(), true);
                if (preShotTimer()) {
                    currentState = SHOOT_PRELOAD;
                }
                break;
            case SHOOT_PRELOAD:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateArtifactGroup1States() {
        switch (currentState) {
            //NEAR ARTIFACT GROUP
            case DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_1:
                follower.followPath(pathSupplier.getLaunchToArtifactGroup1(), true);
                currentState = INTAKE_ARTIFACT_GROUP_1;
                break;
            case INTAKE_ARTIFACT_GROUP_1:
                startIntake();
                follower.followPath(pathSupplier.getIntakeArtifactGroup1(), true);
                currentState = DRIVE_FROM_ARTIFACT_GROUP_1_TO_LAUNCH;
                break;
            case DRIVE_FROM_ARTIFACT_GROUP_1_TO_LAUNCH:
                idleIntake();
                follower.followPath(pathSupplier.getDriveFromArtifactGroup1ToLaunch(), true);
                if (preShotTimer()) {
                    currentState = SHOOT_ARTIFACT_GROUP_1;
                }
                break;
            case SHOOT_ARTIFACT_GROUP_1:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateArtifactGroup2State() {
        switch (currentState) {
            //MIDDLE ARTIFACT GROUP
            case DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_2:
                follower.followPath(pathSupplier.getDriveFromLaunchToArtifactGroup2(), true);
                currentState = INTAKE_ARTIFACT_GROUP_2;
                break;
            case INTAKE_ARTIFACT_GROUP_2:
                startIntake();
                follower.followPath(pathSupplier.getIntakeArtifactGroup2(), true);
                currentState = DRIVE_FROM_ARTIFACT_GROUP_2_TO_LAUNCH;
                break;
            case DRIVE_FROM_ARTIFACT_GROUP_2_TO_LAUNCH:
                idleIntake();
                follower.followPath(pathSupplier.getDriveFromArtifactGroup2ToLaunch(), true);
                if (preShotTimer()) {
                    currentState = SHOOT_ARTIFACT_GROUP_2;
                }
                break;
            case SHOOT_ARTIFACT_GROUP_2:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateArtifactGroup3State() {
        switch (currentState) {
            //FAR ARTIFACT_GROUP
            case DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_3:
                follower.followPath(pathSupplier.getDriveFromLaunchToArtifactGroup3(), true);
                currentState = INTAKE_ARTIFACT_GROUP_3;
                break;
            case INTAKE_ARTIFACT_GROUP_3:
                startIntake();
                follower.followPath(pathSupplier.getIntakeArtifactGroup3(), true);
                currentState = DRIVE_FROM_ARTIFACT_GROUP_3_TO_LAUNCH;
                break;
            case DRIVE_FROM_ARTIFACT_GROUP_3_TO_LAUNCH:
                idleIntake();
                follower.followPath(pathSupplier.getDriveFromArtifactGroup3ToLaunch(), true);
                if (preShotTimer()) {
                    currentState = SHOOT_ARTIFACT_GROUP_3;
                }
                break;
            case SHOOT_ARTIFACT_GROUP_3:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateArtifactGroup4State() {
        switch (currentState) {
            //LOADING ZONE ARTIFACT_GROUP
            case DRIVE_FROM_LAUNCH_TO_ARTIFACT_GROUP_4:
                follower.followPath(pathSupplier.getDriveFromLaunchToArtifactGroup4(), true);
                currentState = INTAKE_ARTIFACT_GROUP_4;
                break;
            case INTAKE_ARTIFACT_GROUP_4:
                startIntake();
                follower.followPath(pathSupplier.getIntakeArtifactGroup4(), true);
                currentState = DRIVE_FROM_ARTIFACT_GROUP_4_TO_LAUNCH;
                break;
            case DRIVE_FROM_ARTIFACT_GROUP_4_TO_LAUNCH:
                idleIntake();
                follower.followPath(pathSupplier.getDriveFromArtifactGroup4ToLaunch(), true);
                if (preShotTimer()) {
                    currentState = SHOOT_ARTIFACT_GROUP_4;
                }
                break;
            case SHOOT_ARTIFACT_GROUP_4:
                shootAndUpdateToNextArtifactGroup();
                break;
        }
    }

    private void updateNoneArtifactGroupState() {
        switch (currentState) {
            case DRIVE_FROM_LAUNCH_TO_END:
                follower.followPath(pathSupplier.getDriveFromLaunchToEnd(), true);
                currentState = ENDING_STATE;
                break;
            case ABORT:
                PathChain pathChain = buildLinearPathChainBetweenPoses(
                    follower, follower.getPose(), getPoseSupplier().getEndPose());
                follower.followPath(pathChain, true);
                currentState = ENDING_STATE;
                break;
            case ENDING_STATE:
                currentState = COMPLETE;
                this.requestOpModeStop();
                break;
        }
    }

    private void shootAndUpdateToNextArtifactGroup() {
        if (readyToShoot()) {
            autoLaunchArtifact();
        }
        if(shootTime != null && shootTime <= getRuntime()) {
            shootTime = null;
            stopLaunchArtifact();
            updateToNextArtifactGroup();
        }
    }

    private void startIntake() {
        hardwareManager.getIntakeMotor().setPower(INTAKE_POWER_IN);
    }

    private void idleIntake() {
        hardwareManager.getIntakeMotor().setPower(INTAKE_IDLE_POWER);
    }

    private void autoLaunch() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if(timedDetection != null && timedDetection.getDetection() != null) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
                targetDistance = targetDetection.ftcPose.range;
                targetBearing = targetDetection.ftcPose.bearing;
                targetYaw = targetDetection.ftcPose.yaw;
                ROBOT_FIELD_X = targetDetection.robotPose.getPosition().x;
                ROBOT_FIELD_Y = targetDetection.robotPose.getPosition().y;
                ROBOT_FIELD_H = targetDetection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                double turretError = calculateBearingToGoal(ROBOT_FIELD_X, ROBOT_FIELD_Y, ROBOT_FIELD_H);
                if (targetDistance > 2.7 && getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
                    turretError -= 6;
                    targetDistance += 0.300;
                } else if (targetDistance > 2.7 && getTargetAprilTag() == AprilTagEnum.RED_TARGET) {
                    turretError -= 2;
                    targetDistance += 0.300;
                }
                flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                telemetry.addData("flywheel RPM", flywheelRPM);

                updateTargetDiff(targetYaw, targetDistance);

                LaunchCalculator.LaunchResult launchResult = LaunchCalculator.getLaunchData(LAUNCH_HEIGHT, TARGET_HEIGHT, targetDistance, flywheelRPM, ROBOT_TARGET_CLOSE_RATE);
                launchVelocity = launchResult.getLaunchVelocity();
                launchAngle = launchResult.getLaunchAngle();
                launchTOF = launchResult.getTOF();
                //if (launchTOF > 0) {
                //    launchLeadAngle = calculateLeadAngle(launchTOF);
                //} else {
                //    launchLeadAngle = 0;
                //}

                setLaunchAngle(launchAngle);

                if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(targetBearing) && targetDetection.id == getTargetAprilTag().getId()) {
                    telemetry.addData("Turret Angle", (targetBearing));
                    telemetry.addData("Target ID", targetDetection.id);

                    //double yawCompensation = targetYaw * 0.1;
                    //turretError = targetBearing + yawCompensation;
                    double setPower = turretBearingPid.calculate(0,turretError) * 0.6;
                    telemetry.addData("Turret Power", setPower);
                    hardwareManager.getTurretMotor().setPower(setPower);
                    tagDetected=true;
                } else {
                    handleNoTagDetected();
                }

                if (launchVelocity > 0 || launchAngle > 0) {
                    launchSolution = true;
                    hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.YELLOW.getLedValue());
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

    private boolean canRotateTurret(double input) {
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()
            || input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()) {
            return false;
        } else {
            return true;
        }
    }

    private double calculateBearingToGoal(double robotX, double robotY, double robotYaw) {

        double goalX = 0;
        double goalY = 0;

        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            goalX = -1.8288;
            goalY = -1.8288;
        } else {
            goalX = -1.8288;
            goalY = 1.8288;
        }

        // Robot field angle to goal
        double absoluteAngleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

        telemetry.addData("Angle to Target", absoluteAngleToGoal);
        // Calculate turret angle to goal
        double relativeAngleToGoal = absoluteAngleToGoal - robotYaw - 90;

        // Normalize
        while (relativeAngleToGoal > 180) relativeAngleToGoal -= 360;
        while (relativeAngleToGoal < -180) relativeAngleToGoal += 360;

        return relativeAngleToGoal * 0.85;

    }

    private void handleNoTagDetected() {
        tagDetected = false;
        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.RED.getLedValue());

        int currentTurretPos = hardwareManager.getTurretMotor().getCurrentPosition();

        double setPower = 0;
        if (Math.abs(currentTurretPos - initialTurretPos) < 6) {

            if (currentTurretPos > initialTurretPos) {
                setPower = -0.3;
            } else if (currentTurretPos < initialTurretPos) {
                setPower = 0.3;
            }
        }
        if (canRotateTurret(setPower) && setPower != 0) {
            hardwareManager.getTurretMotor().setPower(setPower);
        } else {
            hardwareManager.getTurretMotor().setPower(0);
        }

        targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
        hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_CLOSE);
    }

    public boolean readyToShoot() {

        if(isLaunchMotorVelocityWithinThreshold() && isTurretAngleWithinThreshold() && launchSolution) {
            readyToShootCount ++;
        } else {
            readyToShootCount = 0;
        }
        if (readyToShootCount >= MIN_READY_COUNT) {
            readyToShootCount = 0;
            if(shootTime == null) {
                shootTime = getRuntime() + 1.5;
            }
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.YELLOW.getLedValue());
            return true;
        } else {
            return false;
        }
    }

    public boolean preShotTimer() {
        if( preShotTimestamp == null) {
            preShotTimestamp = getRuntime();
        }
        if (preShotTimestamp <= getRuntime()) {
            preShotTimestamp = null;
            return true;
        } else {
            return false;
        }
    }

    private boolean isTurretAngleWithinThreshold() {
        double bearing = Math.abs(turretError);
        if(targetDistance > 2.0) {
            return bearing <= 0.25;
        }
        return bearing <= 0.5;
    }

    private boolean isLaunchMotorVelocityWithinThreshold() {
        double currentRPM = ((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0);
        return Math.abs(targetRPM - currentRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
    }

    private void setLaunchAngle(double setAngle) {
        if (setAngle != 0) {
            double positionValue = Range.clip(((MAX_LAUNCH_ANGLE - setAngle) * (0.65/((MAX_LAUNCH_ANGLE-MIN_LAUNCH_ANGLE)))), 0.0, 0.65);
            //telemetry.addData("servo value", positionValue);
            hardwareManager.getAngleServo().setPosition(positionValue);
        }
    }

    private void autoLaunchArtifact() {
        hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_OPEN);
        startIntake();
    }

    private void stopLaunchArtifact() {
        hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_CLOSE);
        idleIntake();
    }

    private void updateTelemetry() {
        updateRuntime();
        telemetry.addData("Tag detected", tagDetected);
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
    abstract AbstractPoseSupplier getPoseSupplier();
}