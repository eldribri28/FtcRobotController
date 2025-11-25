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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
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

@Autonomous(name="auto drive test")
public class AutonDriveTest extends LinearOpMode {
    private HardwareManager hardwareManager;
    private MecanumDrive drive;
    private Pose2d currentPose;
    private AprilTagEngine aprilTagEngine;
    private ArtifactColorEnum launcherArtifactColor = ArtifactColorEnum.UNKNOWN;
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private ColorManager colorManager;
    private AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }
    private double targetRPM = 0;
    private double actualRPM = 0;
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    private AutonomousStateEnum currentState = AutonomousStateEnum.FIRE_ALL_ARTIFACTS;
    private ArtifactBundleEnum currentArtifactBundle = ArtifactBundleEnum.PRELOAD;

    public enum ArtifactBundleEnum {
        PRELOAD,
        FIELD_SET_1,
        FIELD_SET_2,
        FIELD_SET_3,
        NONE
    }

    public enum AutonomousStateEnum {
        FIRE_ALL_ARTIFACTS,
        DRIVE_TO_FIELD_SET,
        INTAKE_ARTIFACTS,
        RETURN_TO_LAUNCH,
        EXIT_LAUNCH
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread aprilTagEngineThread = new Thread(aprilTagEngine);
        while(!isStarted()) {
            updateArtifactColors();
        }
        resetRuntime();
        while(opModeIsActive()) {
            try {
                updateArtifactColors();
                if(currentState == AutonomousStateEnum.FIRE_ALL_ARTIFACTS) {
                   fireAllArtifacts();
                   if(currentArtifactBundle == ArtifactBundleEnum.PRELOAD) {
                       updateStates(AutonomousStateEnum.DRIVE_TO_FIELD_SET, ArtifactBundleEnum.FIELD_SET_1);
                   } else if (currentArtifactBundle == ArtifactBundleEnum.FIELD_SET_1) {
                       updateStates(AutonomousStateEnum.DRIVE_TO_FIELD_SET, ArtifactBundleEnum.FIELD_SET_2);
                   } else if (currentArtifactBundle == ArtifactBundleEnum.FIELD_SET_2) {
                       updateStates(AutonomousStateEnum.DRIVE_TO_FIELD_SET, ArtifactBundleEnum.FIELD_SET_3);
                   } else {
                       updateStates(AutonomousStateEnum.EXIT_LAUNCH, ArtifactBundleEnum.NONE);
                   }
                } else if(currentState == AutonomousStateEnum.DRIVE_TO_FIELD_SET) {
                    driveToFieldSet();
                } else if(currentState == AutonomousStateEnum.INTAKE_ARTIFACTS) {
                    intakeArtifacts();
                } else if(currentState == AutonomousStateEnum.RETURN_TO_LAUNCH) {
                    returnToLaunch();
                } else if (currentState == AutonomousStateEnum.EXIT_LAUNCH) {
                    exitLaunch();
                }
            } finally {
                scheduler.shutdownNow();
            }
        }
    }

    private void updateArtifactColors() {
        colorManager.setArtifactColors();
        launcherArtifactColor = colorManager.getLauncherArtifactColor();
    }

    private void fireAllArtifacts() {
        while (artifactInLauncher()) {
            Actions.runBlocking(
                    new SequentialAction(
                        new LockOnTarget(),
                        new Shoot(),
                        new RunIntake(500)));
        }
    }

    private void driveToFieldSet() {

    }

    private void intakeArtifacts() {

    }

    private void returnToLaunch() {

    }

    private void exitLaunch() {

    }

    private void updateStates(AutonomousStateEnum autonomousStateEnum, ArtifactBundleEnum artifactBundleEnum) {
        this.currentState = autonomousStateEnum;
        this.currentArtifactBundle = artifactBundleEnum;
    }

    private boolean artifactInLauncher() {
        return launcherArtifactColor != NONE;
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        colorManager = new ColorManager(hardwareManager);
        //TODO: update initial pose based on field coordinates
        currentPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, currentPose);
    }
    public class LockOnTarget implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            TimedAprilTagDetection timedAprilTagDetection = aprilTagEngine.getTimedTargetDetection();
            if(timedAprilTagDetection != null && timedAprilTagDetection.getDetection() != null) {
                long detectionAge = timedAprilTagDetection.getAgeInMillis();
                AprilTagDetection detection = timedAprilTagDetection.getDetection();
                telemetryPacket.addLine("Target detection age(millisecond): " + detectionAge);
                if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
                    if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(detection.ftcPose.bearing)) {
                        telemetryPacket.addLine("Turret Angle: " + detection.ftcPose.bearing);
                        double setPower = turretBearingPid.calculate(1, detection.ftcPose.bearing);
                        telemetryPacket.addLine("Turret Power: " +  setPower);
                        hardwareManager.getTurretMotor().setPower(setPower);
                    } else {
                        hardwareManager.getTurretMotor().setPower(0);
                    }

                    double targetDistance = detection.ftcPose.range;
                    targetRPM = calculateTargetRPM(targetDistance);
                    LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult(actualRPM, targetDistance);
                    if(launchResult != null) {
                        setLaunchAngle(launchResult.getLaunchAngle());
                    }
                    hardwareManager.getLauncherMotor().setVelocity(calculateTargetVelocity(targetRPM));
                } else {
                    hardwareManager.getTurretMotor().setPower(0);
                }
            } else {
                //TODO add logic to try to find april tag
            }
            return false;
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

    private double getActualRPM() {
        return (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
    }

    private double calculateTargetRPM(double targetDistance) {
        return Math.round(((targetDistance / 1.670) * 900.0) + 1800.0);
    }

    private double calculateTargetVelocity(double targetRPM) {
        return ((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0);
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((70.0 - launchAngle) / 35.0));
        if(positionValue >= 0 && positionValue <= 0.8) {
            hardwareManager.getAngleServo().setPosition(positionValue);
        }
    }

    public class Shoot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            actualRPM = getActualRPM();
            while(Math.abs(targetRPM - actualRPM) >= 50 && opModeIsActive()) {
                sleep(20);
                actualRPM = getActualRPM();
            }
            if(opModeIsActive()) {
                hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_UP);
                Runnable servoDown = () -> {
                    hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
                };
                scheduler.schedule(servoDown, 100, TimeUnit.MILLISECONDS);
            }
            return false;
        }
    }

    public class RunIntake implements Action {
        private final long millisecondsToRun;

        public RunIntake(long millisecondsToRun) {
            this.millisecondsToRun = millisecondsToRun;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hardwareManager.getIntakeMotor().setPower(0);
            telemetryPacket.addLine("Intake Started");
            Runnable stopIntake = () -> {
                hardwareManager.getIntakeMotor().setPower(0);
            };
            scheduler.schedule(stopIntake, millisecondsToRun, TimeUnit.MILLISECONDS);
            return false;
        }
    }

    public class GetArtifactColors implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            colorManager.setArtifactColors();
            launcherArtifactColor = colorManager.getLauncherArtifactColor();
            telemetryPacket.addLine("launchColorSensor1 Color:" + launcherArtifactColor.name());
            return false;
        }
    }
}
