package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

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
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "auto drive test- trajectory")
public class AutonDriveTest3 extends LinearOpMode {
    private HardwareManager hardwareManager;
    private MecanumDrive drive;
    private AprilTagEngine aprilTagEngine;
    private ColorManager colorManager;
    private AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }
    private StartPositionEnum getStartPosition() {
        return StartPositionEnum.FAR;
    }
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    private Thread colorManagerThread;
    private Thread aprilTagEngineThread;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();
            waitForStart();
            resetRuntime();
            aprilTagEngineThread.start();
            Actions.runBlocking(
                    new ParallelAction(
                            getDriveAction(),
                            new UpdateTelemetry()));
        } finally {
            scheduler.shutdownNow();
            if (aprilTagEngineThread != null) {
                aprilTagEngineThread.interrupt();
                aprilTagEngine.teardown();
            }
            if (colorManagerThread != null) {
                colorManagerThread.interrupt();
            }
            telemetry.update();
        }
    }

    private Action getDriveAction() {
        Pose2d startPose = FieldLocation.START_LOCATION
                .getLocation(getStartPosition(), getTargetAprilTag());
        Pose2d firstArtifacts = FieldLocation.FIELD_SET_1_LOCATION
                .getLocation(getStartPosition(), getTargetAprilTag());
        Pose2d secondArtifacts = FieldLocation.FIELD_SET_2_LOCATION
                .getLocation(getStartPosition(), getTargetAprilTag());
        Pose2d thirdArtifacts = FieldLocation.FIELD_SET_3_LOCATION
                .getLocation(getStartPosition(), getTargetAprilTag());
        Pose2d fourthArtifacts = FieldLocation.FIELD_SET_4_LOCATION
                .getLocation(getStartPosition(), getTargetAprilTag());
        Pose2d launchNearStart = FieldLocation.LAUNCH_NEAR_START_LOCATION
                .getLocation(getStartPosition(), getTargetAprilTag());
        Pose2d launchAwayFromStart = FieldLocation.LAUNCH_AWAY_FROM_START_LOCATION
                .getLocation(getStartPosition(), getTargetAprilTag());
        Pose2d endLocation = FieldLocation.END_LOCATION
                .getLocation(getStartPosition(), getTargetAprilTag());
        double endYForArtifactPickup = getTargetAprilTag() == AprilTagEnum.BLUE_TARGET ? -60 : 60;
        double endForYArtifactPickupGroup3 = getTargetAprilTag() == AprilTagEnum.BLUE_TARGET ? -54 : 54;
        return drive.actionBuilder(startPose)
                .strafeTo(launchNearStart.component1())
                .turnTo(launchNearStart.component2())
                //ARTIFACTS 1
                .strafeTo(firstArtifacts.component1())
                .turnTo(firstArtifacts.component2())
                .lineToY(endYForArtifactPickup)
                .strafeTo(launchNearStart.component1())
                .turnTo(launchNearStart.component2())
                //ARTIFACTS 4
                .turnTo(fourthArtifacts.component2())
                .strafeTo(fourthArtifacts.component1())
                .waitSeconds(.1)
                .lineToX(62)
                .waitSeconds(.1)
                .strafeTo(launchNearStart.component1())
                .turnTo(launchNearStart.component2())
                //ARTIFACTS 2
                .strafeTo(secondArtifacts.component1())
                .turnTo(secondArtifacts.component2())
                .lineToY(endYForArtifactPickup)
                .strafeTo(launchNearStart.component1())
                .turnTo(launchNearStart.component2())
                //ARTIFACTS 3
                .strafeTo(thirdArtifacts.component1())
                .turnTo(thirdArtifacts.component2())
                .lineToY(endForYArtifactPickupGroup3)
                .strafeTo(launchNearStart.component1())
                .turnTo(launchNearStart.component2())
                //END
                .strafeTo(endLocation.component1())
                .turnTo(endLocation.component2())
                .build();
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        aprilTagEngineThread = new Thread(aprilTagEngine);
        colorManager = new ColorManager(hardwareManager);
        colorManagerThread = new Thread(colorManager);
        colorManagerThread.start();
        drive = new MecanumDrive(
                hardwareMap,
                FieldLocation.START_LOCATION
                        .getLocation(getStartPosition(), getTargetAprilTag()));
    }

    private class UpdateTelemetry implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.addLine("--------------------------");
            telemetryPacket.addLine("*****APRILTAG ENGINE*****");
            aprilTagEngine.getTelemetry().forEach(telemetryPacket::put);
            telemetryPacket.addLine("*****COLOR MANAGER*****");
            colorManager.getTelemetry().forEach(telemetryPacket::put);
            return false;
        }
    }

    public enum FieldLocation {
        FIELD_SET_1_LOCATION(
                new Pose2d(-11.5, 27, Math.toRadians(90)),
                new Pose2d(35, 27, Math.toRadians(90)),
                new Pose2d(-11.5, -27, Math.toRadians(270)),
                new Pose2d(35, -27, Math.toRadians(270))
        ),
        FIELD_SET_2_LOCATION(
                new Pose2d(12,27, Math.toRadians(90)),
                new Pose2d(12,27, Math.toRadians(90)),
                new Pose2d(12,-27, Math.toRadians(270)),
                new Pose2d(12,-27, Math.toRadians(270))
        ),
        FIELD_SET_3_LOCATION(
                new Pose2d(35.5, 27, Math.toRadians(90)),
                new Pose2d(-11.5, 27, Math.toRadians(90)),
                new Pose2d(35.5, -27, Math.toRadians(270)),
                new Pose2d(-11.5, -27, Math.toRadians(270))
        ),
        FIELD_SET_4_LOCATION(
                new Pose2d(1,1,1),
                new Pose2d(1,1,1),
                new Pose2d(1,1,1),
                new Pose2d(40, -65, Math.toRadians(0))
        ),
        END_LOCATION(
                new Pose2d(0,50, Math.toRadians(90)),
                new Pose2d(0,50, Math.toRadians(90)),
                new Pose2d(0,-50, Math.toRadians(270)),
                new Pose2d(0,-50, Math.toRadians(270))
        ),
        LAUNCH_NEAR_START_LOCATION(
                new Pose2d(-25, 25, Math.toRadians(90)),
                new Pose2d(50,10, Math.toRadians(90)),
                new Pose2d(-25,-25, Math.toRadians(270)),
                new Pose2d(50,-10, Math.toRadians(270))
        ),
        LAUNCH_AWAY_FROM_START_LOCATION(
                new Pose2d(50,10, Math.toRadians(90)),
                new Pose2d(-25,25, Math.toRadians(90)),
                new Pose2d(50,-10, Math.toRadians(270)),
                new Pose2d(-25,-25, Math.toRadians(270))
        ),
        START_LOCATION(
                new Pose2d(-49.5, 49.5, Math.toRadians(127)),
                new Pose2d(63, 17.3, Math.toRadians(90)),
                new Pose2d(-49.5, -49.5, Math.toRadians(233)),
                new Pose2d(63, -17.3, Math.toRadians(270))
        );


        FieldLocation(Pose2d redNearLocation, Pose2d redFarLocation, Pose2d blueNearLocation, Pose2d blueFarLocation) {
            this.redNearLocation = redNearLocation;
            this.redFarLocation = redFarLocation;
            this.blueNearLocation = blueNearLocation;
            this.blueFarLocation = blueFarLocation;
        }

        private final Pose2d redNearLocation;
        private final Pose2d redFarLocation;
        private final Pose2d blueNearLocation;
        private final Pose2d blueFarLocation;

        public Pose2d getLocation(StartPositionEnum startPositionEnum, AprilTagEnum targetAprilTag) {
            Pose2d location;
            if(targetAprilTag == AprilTagEnum.BLUE_TARGET) {
                if(startPositionEnum == StartPositionEnum.FAR) {
                    location = blueFarLocation;
                } else {
                    location = blueNearLocation;
                }
            } else {
                if(startPositionEnum == StartPositionEnum.FAR) {
                    location = redFarLocation;
                } else {
                    location = redNearLocation;
                }
            }
            return location;
        }
    }
}
