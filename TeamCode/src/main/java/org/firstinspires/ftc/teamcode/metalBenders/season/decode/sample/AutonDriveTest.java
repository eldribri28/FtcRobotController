package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.NONE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.ColorManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name="auto drive test")
public class AutonDriveTest {//extends LinearOpMode {
//    private HardwareManager hardwareManager;
//    private MecanumDrive drive;
//    private Pose2d currentPose;
//    private AprilTagEngine aprilTagEngine;
//    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
//    private ColorManager colorManager;
//    private AprilTagEnum getTargetAprilTag() {
//        return AprilTagEnum.BLUE_TARGET;
//    }
//    private StartPositionEnum getStartPosition() {
//        return StartPositionEnum.FAR;
//    };
//    private double targetRPM = 0;
//    private double actualRPM = 0;
//    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
//    private AutonomousStateEnum currentState = AutonomousStateEnum.MOVE_TO_LAUNCH;
//    private ArtifactGroupEnum currentArtifactGroup = ArtifactGroupEnum.PRELOAD;
//    private Thread colorManagerThread;
//    private Thread aprilTagEngineThread;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        try {
//            initialize();
//            waitForStart();
//            resetRuntime();
//            aprilTagEngineThread.start();
//            while (opModeIsActive()) {
//                checkAbortAndExitLaunch();
//                if (currentState == AutonomousStateEnum.FIRE_ALL_ARTIFACTS) {
//                    fireAllArtifacts();
//                    if (currentArtifactGroup == ArtifactGroupEnum.PRELOAD) {
//                        updateStateAndTargetArtifactGroup(AutonomousStateEnum.DRIVE_TO_FIELD_SET, ArtifactGroupEnum.FIELD_SET_1);
//                    } else if (currentArtifactGroup == ArtifactGroupEnum.FIELD_SET_1) {
//                        updateStateAndTargetArtifactGroup(AutonomousStateEnum.DRIVE_TO_FIELD_SET, ArtifactGroupEnum.FIELD_SET_2);
//                    } else if (currentArtifactGroup == ArtifactGroupEnum.FIELD_SET_2) {
//                        updateStateAndTargetArtifactGroup(AutonomousStateEnum.DRIVE_TO_FIELD_SET, ArtifactGroupEnum.FIELD_SET_3);
//                    } else {
//                        updateStateAndTargetArtifactGroup(AutonomousStateEnum.EXIT_LAUNCH, ArtifactGroupEnum.NONE);
//                    }
//                } else if (currentState == AutonomousStateEnum.DRIVE_TO_FIELD_SET) {
//                    driveToFieldSet();
//                } else if (currentState == AutonomousStateEnum.INTAKE_ARTIFACTS) {
//                    intakeArtifacts();
//                } else if (currentState == AutonomousStateEnum.MOVE_TO_LAUNCH) {
//                    moveToLaunch();
//                } else if (currentState == AutonomousStateEnum.EXIT_LAUNCH) {
//                    driveToEndLocation();
//                }
//                updateTelemetry();
//            }
//        } finally {
//            scheduler.shutdownNow();
//            if(aprilTagEngineThread != null) {
//                aprilTagEngineThread.interrupt();
//                aprilTagEngine.teardown();
//            }
//            if(colorManagerThread != null) {
//                colorManagerThread.interrupt();
//            }
//            telemetry.update();
//        }
//    }
//
//    private void updateTelemetry() {
//        telemetry.addData("Current State", currentState.name());
//        telemetry.addData("Current Artifact Group", currentArtifactGroup.name());
//        telemetry.addLine("--------------------------");
//        telemetry.addLine("*****APRILTAG ENGINE*****");
//        aprilTagEngine.getTelemetry().forEach((k, v) -> telemetry.addData(k, v));
//        telemetry.addLine("*****COLOR MANAGER*****");
//        colorManager.getTelemetry().forEach((k, v) -> telemetry.addData(k, v));
//        telemetry.update();
//    }
//
//    private void checkAbortAndExitLaunch() {
//        if(getRuntime() > 25) {
//            updateStateAndTargetArtifactGroup(AutonomousStateEnum.EXIT_LAUNCH, ArtifactGroupEnum.NONE);
//        }
//    }
//
//    private void fireAllArtifacts() {
//        while (colorManager.getLauncherArtifactColor() != NONE) {
//            Actions.runBlocking(
//                    new SequentialAction(
//                        new LockOnTarget(),
//                        new Shoot()));
//        }
//    }
//
//    private void driveToFieldSet() {
//        Pose2d destination = currentPose;
//        if(currentArtifactGroup == ArtifactGroupEnum.FIELD_SET_1) {
//            destination = FieldLocation.FIELD_SET_1_LOCATION
//                    .getLocation(getStartPosition(), getTargetAprilTag());
//        } else if (currentArtifactGroup == ArtifactGroupEnum.FIELD_SET_2) {
//            destination = FieldLocation.FIELD_SET_2_LOCATION
//                    .getLocation(getStartPosition(), getTargetAprilTag());
//        } else if (currentArtifactGroup == ArtifactGroupEnum.FIELD_SET_3) {
//            destination = FieldLocation.FIELD_SET_3_LOCATION
//                    .getLocation(getStartPosition(), getTargetAprilTag());
//        }
//        Actions.runBlocking(
//                drive.actionBuilder(currentPose)
//                        .splineToLinearHeading(destination, destination.heading)
//                        .build());
//        currentPose = destination;
//        currentState = AutonomousStateEnum.INTAKE_ARTIFACTS;
//    }
//
//    private void intakeArtifacts() {
//        hardwareManager.getIntakeMotor().setPower(1.0);
//        Actions.runBlocking(
//                drive.actionBuilder(currentPose)
//                        .lineToY(25.0).lineToY(-25.0).build()
//        );
//        hardwareManager.getIntakeMotor().setPower(0.0);
//        currentState = AutonomousStateEnum.MOVE_TO_LAUNCH;
//    }
//
//    private void moveToLaunch() {
//        Pose2d launchLocation =
//                FieldLocation.LAUNCH_LOCATION.getLocation(getStartPosition(), getTargetAprilTag());
//        Actions.runBlocking(
//                drive.actionBuilder(currentPose)
//                        .splineToLinearHeading(launchLocation, launchLocation.heading)
//                        .build()
//        );
//        currentPose = launchLocation;
//        updateState(AutonomousStateEnum.FIRE_ALL_ARTIFACTS);
//    }
//
//    private void driveToEndLocation() {
//        Pose2d endLocation =
//                FieldLocation.END_LOCATION.getLocation(getStartPosition(), getTargetAprilTag());
//        Actions.runBlocking(
//                drive.actionBuilder(currentPose)
//                        .splineToLinearHeading(endLocation, endLocation.heading)
//                        .build()
//        );
//        currentPose = endLocation;
//        updateState(AutonomousStateEnum.STOP_AND_WAIT);
//    }
//
//    private void updateState(AutonomousStateEnum autonomousStateEnum) {
//        updateStateAndTargetArtifactGroup(autonomousStateEnum, currentArtifactGroup);
//    }
//
//    private void updateStateAndTargetArtifactGroup(AutonomousStateEnum autonomousStateEnum, ArtifactGroupEnum artifactGroupEnum) {
//        this.currentState = autonomousStateEnum;
//        this.currentArtifactGroup = artifactGroupEnum;
//    }
//
//    private void initialize() {
//        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
//        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
//        aprilTagEngineThread = new Thread(aprilTagEngine);
//        colorManager = new ColorManager(hardwareManager);
//        colorManagerThread = new Thread(colorManager);
//        colorManagerThread.start();
//        currentPose = FieldLocation.START_LOCATION.getLocation(getStartPosition(), getTargetAprilTag());
//        drive = new MecanumDrive(hardwareMap, currentPose);
//    }
//
//    public class LockOnTarget implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            TimedAprilTagDetection timedAprilTagDetection = aprilTagEngine.getTimedTargetDetection();
//            if(timedAprilTagDetection != null && timedAprilTagDetection.getDetection() != null) {
//                long detectionAge = timedAprilTagDetection.getAgeInMillis();
//                AprilTagDetection detection = timedAprilTagDetection.getDetection();
//                telemetryPacket.addLine("Target detection age(millisecond): " + detectionAge);
//                if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
//                    if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(detection.ftcPose.bearing)) {
//                        telemetryPacket.addLine("Turret Angle: " + detection.ftcPose.bearing);
//                        double setPower = turretBearingPid.calculate(1, detection.ftcPose.bearing);
//                        telemetryPacket.addLine("Turret Power: " +  setPower);
//                        hardwareManager.getTurretMotor().setPower(setPower);
//                    } else {
//                        hardwareManager.getTurretMotor().setPower(0);
//                    }
//
//                    double targetDistance = detection.ftcPose.range;
//                    targetRPM = calculateTargetRPM(targetDistance);
//                    LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult(actualRPM, targetDistance);
//                    if(launchResult != null) {
//                        setLaunchAngle(launchResult.getLaunchAngle());
//                    }
//                    hardwareManager.getLauncherMotor().setVelocity(calculateTargetVelocity(targetRPM));
//                } else {
//                    hardwareManager.getTurretMotor().setPower(0);
//                }
//            }
//            return false;
//        }
//    }
//
//    private boolean canRotateTurret(double input) {
//        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()) {
//            return false;
//        } else if (input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()){
//            return false;
//        } else {
//            return true;
//        }
//    }
//
//    private double getActualRPM() {
//        return (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
//    }
//
//    private double calculateTargetRPM(double targetDistance) {
//        return Math.round(((targetDistance / 1.670) * 900.0) + 1800.0);
//    }
//
//    private double calculateTargetVelocity(double targetRPM) {
//        return ((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0);
//    }
//
//    private void setLaunchAngle(double launchAngle) {
//        double positionValue = Math.abs(((70.0 - launchAngle) / 35.0));
//        if(positionValue >= 0 && positionValue <= 0.8) {
//            hardwareManager.getAngleServo().setPosition(positionValue);
//        }
//    }
//
//    public class Shoot implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            actualRPM = getActualRPM();
//            while(Math.abs(targetRPM - actualRPM) >= 50 && opModeIsActive()) {
//                sleep(20);
//                actualRPM = getActualRPM();
//            }
//            if(opModeIsActive()) {
//                hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_UP);
//                Runnable servoDown = () -> {
//                    hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
//                };
//                scheduler.schedule(servoDown, 100, TimeUnit.MILLISECONDS);
//            }
//            return false;
//        }
//    }
//
//    public enum FieldLocation {
//        FIELD_SET_1_LOCATION(
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1)
//        ),
//        FIELD_SET_2_LOCATION(
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1)
//        ),
//        FIELD_SET_3_LOCATION(
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1)
//        ),
//        END_LOCATION(
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1)
//        ),
//        LAUNCH_LOCATION(
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1)
//        ),
//        START_LOCATION(
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1),
//                new Pose2d(1,1,1)
//        );
//
//
//        FieldLocation(Pose2d redNearLocation, Pose2d redFarLocation, Pose2d blueNearLocation, Pose2d blueFarLocation) {
//            this.redNearLocation = redNearLocation;
//            this.redFarLocation = redFarLocation;
//            this.blueNearLocation = blueNearLocation;
//            this.blueFarLocation = blueFarLocation;
//        }
//
//        private final Pose2d redNearLocation;
//        private final Pose2d redFarLocation;
//        private final Pose2d blueNearLocation;
//        private final Pose2d blueFarLocation;
//
//        public Pose2d getLocation(StartPositionEnum startPositionEnum, AprilTagEnum aprilTagEnum) {
//            Pose2d location;
//            if(aprilTagEnum == AprilTagEnum.BLUE_TARGET) {
//                if(startPositionEnum == StartPositionEnum.FAR) {
//                    location = blueFarLocation;
//                } else {
//                    location = blueNearLocation;
//                }
//            } else {
//                if(startPositionEnum == StartPositionEnum.FAR) {
//                    location = redFarLocation;
//                } else {
//                    location = redNearLocation;
//                }
//            }
//            return location;
//        }
//    }
//
//    public enum ArtifactGroupEnum {
//        NONE,
//        PRELOAD,
//        FIELD_SET_1,
//        FIELD_SET_2,
//        FIELD_SET_3
//    }
//
//    public enum AutonomousStateEnum {
//        FIRE_ALL_ARTIFACTS,
//        DRIVE_TO_FIELD_SET,
//        INTAKE_ARTIFACTS,
//        MOVE_TO_LAUNCH,
//        EXIT_LAUNCH,
//        STOP_AND_WAIT
//    }
}
