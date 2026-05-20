package org.firstinspires.ftc.teamcode.pedroPathing.autonomous; // make sure this aligns with class location

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
import static org.firstinspires.ftc.teamcode.pedroPathing.autonomous.ArtifactGroup.FAR;
import static org.firstinspires.ftc.teamcode.pedroPathing.autonomous.ArtifactGroup.LOADING_ZONE;
import static org.firstinspires.ftc.teamcode.pedroPathing.autonomous.ArtifactGroup.MIDDLE;
import static org.firstinspires.ftc.teamcode.pedroPathing.autonomous.ArtifactGroup.PRELOAD;
import static org.firstinspires.ftc.teamcode.pedroPathing.autonomous.AutonomousStateEnum.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.pose.PoseUtil.buildLinearPathChainBetweenTwoPoses;
import static org.firstinspires.ftc.teamcode.pedroPathing.pose.PoseUtil.buildLinearPathChainOutAndBack;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.pedroPathing.pose.AbstractPoseSupplier;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Example Auto", group = "Examples")
public class BaseAuto extends LinearOpMode {
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
    boolean firing = false;
    private HardwareManager hardwareManager;
    private AprilTagEngine aprilTagEngine;
    private Thread aprilTagEngineThread;
    private Follower follower;
    private AutonomousStateEnum currentState = SHOOT_PRELOAD;
    private PathChain
            startToShoot, shootToNearArtifactGroup, intakeNearArtifactGroup, nearArtifactGroupToNearShoot,
            shootToMiddleArtifactGroup, intakeMiddleArtifactGroup, middleArtifactGroupToNearShoot,
            shootToFarArtifactGroup, intakeFarArtifactGroup, farArtifactGroupToFarShoot, farShootToEnd;
    private List<ArtifactGroup> ARTIFACT_GROUP_ORDER = new ArrayList<>(Arrays.asList(PRELOAD, NEAR, LOADING_ZONE, MIDDLE, FAR));

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();
            hardwareManager.getAngleServo().setPosition(0);
            waitForStart();
            resetRuntime();
            aprilTagEngineThread.start();
            hardwareManager.postStartInitialization();
            while (opModeIsActive()) {
                telemetry.addData("currentState", currentState.name());
                telemetry.addData("follower.isBusy", follower.isBusy());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                updateState();
                autoLaunch();
                follower.update();
                telemetry.update();
            }
        } finally {
            if(aprilTagEngineThread != null) {
                aprilTagEngineThread.interrupt();
                aprilTagEngine.teardown();
            }
            telemetry.update();
        }
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        aprilTagEngineThread = new Thread(aprilTagEngine);
        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.RED.getLedValue());
        hardwareManager.getIntakeServo().setPosition(INTAKE_DOWN);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        if(getStartPosition() == NEAR) {
            currentState = DRIVE_FROM_START_TO_LAUNCH;
        } else {
            currentState = SHOOT_PRELOAD;
        }
    }

    private void updateState() {
        if(!follower.isBusy()) {
            switch (currentState) {

                //PRELOAD STATES
                case DRIVE_FROM_START_TO_LAUNCH:
                    follower.followPath(startToShoot, true);
                    currentState = SHOOT_PRELOAD;
                    break;
                case SHOOT_PRELOAD:
                    if(readyToShoot()) {
                        autoLaunchArtifact();
                        sleep(1000);
                        stopLaunchArtifact();
                        currentState = DRIVE_FROM_NEAR_LAUNCH_TO_NEAR_ARTIFACT_GROUP;
                    }
                   break;

                //NEAR ARTIFACT GROUP
                case DRIVE_FROM_NEAR_LAUNCH_TO_NEAR_ARTIFACT_GROUP:
                    follower.followPath(shootToNearArtifactGroup, true);
                    currentState = INTAKE_NEAR_ARTIFACT_GROUP;
                    break;
                case INTAKE_NEAR_ARTIFACT_GROUP:
                    startIntake();
                    follower.followPath(intakeNearArtifactGroup);
                    currentState = DRIVE_FROM_NEAR_ARTIFACT_GROUP_TO_NEAR_LAUNCH;
                    break;
                case DRIVE_FROM_NEAR_ARTIFACT_GROUP_TO_NEAR_LAUNCH:
                    stopIntake();
                    follower.followPath(nearArtifactGroupToNearShoot);
                    sleep(1000);
                    currentState = SHOOT_NEAR_ARTIFACT_GROUP;
                    break;
                case SHOOT_NEAR_ARTIFACT_GROUP:
                    if(readyToShoot()) {
                        autoLaunchArtifact();
                        sleep(1000);
                        stopLaunchArtifact();
                        currentState = DRIVE_FROM_NEAR_LAUNCH_TO_MIDDLE_ARTIFACT_GROUP;
                    }
                    break;

                //MIDDLE ARTIFACT GROUP
                case DRIVE_FROM_NEAR_LAUNCH_TO_MIDDLE_ARTIFACT_GROUP:
                    follower.followPath(shootToMiddleArtifactGroup, true);
                    currentState = INTAKE_MIDDLE_ARTIFACT_GROUP;
                    break;
                case INTAKE_MIDDLE_ARTIFACT_GROUP:
                    startIntake();
                    follower.followPath(intakeMiddleArtifactGroup);
                    currentState = DRIVE_FROM_MIDDLE_ARTIFACT_GROUP_TO_NEAR_LAUNCH;
                    break;
                case DRIVE_FROM_MIDDLE_ARTIFACT_GROUP_TO_NEAR_LAUNCH:
                    stopIntake();
                    follower.followPath(middleArtifactGroupToNearShoot);
                    sleep(1000);
                    currentState = SHOOT_MIDDLE_ARTIFACT_GROUP;
                    break;
                case SHOOT_MIDDLE_ARTIFACT_GROUP:
                    if(readyToShoot()) {
                        autoLaunchArtifact();
                        sleep(1000);
                        stopLaunchArtifact();
                        currentState = DRIVE_FROM_NEAR_LAUNCH_TO_FAR_ARTIFACT_GROUP;
                    }
                    break;

                //FAR ARTIFACT_GROUP
                case DRIVE_FROM_NEAR_LAUNCH_TO_FAR_ARTIFACT_GROUP:
                    follower.followPath(shootToFarArtifactGroup, true);
                    currentState = INTAKE_FAR_ARTIFACT_GROUP;
                    break;
                case INTAKE_FAR_ARTIFACT_GROUP:
                    startIntake();
                    follower.followPath(intakeFarArtifactGroup);
                    currentState = DRIVE_FROM_FAR_ARTIFACT_GROUP_TO_FAR_LAUNCH;
                    break;
                case DRIVE_FROM_FAR_ARTIFACT_GROUP_TO_FAR_LAUNCH:
                    stopIntake();
                    follower.followPath(farArtifactGroupToFarShoot);
                    sleep(1000);
                    currentState = SHOOT_FAR_ARTIFACT_GROUP;
                    break;
                case SHOOT_FAR_ARTIFACT_GROUP:
                    if(readyToShoot()) {
                        autoLaunchArtifact();
                        sleep(1000);
                        stopLaunchArtifact();
                        currentState = DRIVE_FROM_FAR_LAUNCH_TO_END;
                    }
                    break;
                case DRIVE_FROM_FAR_LAUNCH_TO_END:
                    follower.followPath(farShootToEnd);
                    currentState = END_STATE;
                    break;
            }
        }
    }

    private void buildPaths() {
        AbstractPoseSupplier poseSupplier =
            AbstractPoseSupplier.getPoseSupplier(getStartPosition(), getTargetAprilTag());
        startToShoot = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getStartPose(), poseSupplier.getNearLaunchPose());
        shootToNearArtifactGroup = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getStartPose(), poseSupplier.getNearArtifactGroupPose());
        nearArtifactGroupToNearShoot = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getNearArtifactGroupPose(), poseSupplier.getNearLaunchPose());
        intakeNearArtifactGroup = buildLinearPathChainOutAndBack(
            follower, poseSupplier.getNearArtifactGroupPose(), poseSupplier.getNearArtifactEndIntakePose());
        shootToMiddleArtifactGroup = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getNearLaunchPose(), poseSupplier.getMiddleArtifactGroupPose());
        middleArtifactGroupToNearShoot = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getMiddleArtifactGroupPose(), poseSupplier.getNearLaunchPose());
        intakeMiddleArtifactGroup = buildLinearPathChainOutAndBack(
            follower, poseSupplier.getMiddleArtifactGroupPose(), poseSupplier.getMiddleArtifactEndIntakePose());
        shootToFarArtifactGroup = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getNearLaunchPose(), poseSupplier.getFarArtifactGroupPose());
        farArtifactGroupToFarShoot = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getFarArtifactGroupPose(), poseSupplier.getFarLaunchPose());
        intakeFarArtifactGroup = buildLinearPathChainOutAndBack(
            follower, poseSupplier.getFarArtifactGroupPose(), poseSupplier.getFarArtifactEndIntakePose());
        farShootToEnd = buildLinearPathChainBetweenTwoPoses(
            follower, poseSupplier.getFarLaunchPose(), poseSupplier.getEndPose());
        follower.setStartingPose(poseSupplier.getStartPose());
    }

    private void startIntake() {
        hardwareManager.getIntakeMotor().setPower(INTAKE_POWER_IN);
    }

    private void stopIntake() {
        hardwareManager.getIntakeMotor().setPower(INTAKE_NO_POWER);
    }

    private AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }

    private StartPositionEnum getStartPosition() {
        return StartPositionEnum.FAR;
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
                    double setPower = turretBearingPid.calculate(0,turretError) * 0.5;
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
                    setNoTagDetected();
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
                setNoTagDetected();
            }
        } else {
            setNoTagDetected();
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
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()) {
            return false;
        } else if (input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()){
            return false;
        } else {
            return true;
        }
    }

    private void setNoTagDetected() {
        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.RED.getLedValue());
        targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
        hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_CLOSE);
        setLedStates(NO_TAG_DETECTED);
    }

    public boolean readyToShoot() {
        return isLaunchMotorVelocityWithinThreshold()
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
}