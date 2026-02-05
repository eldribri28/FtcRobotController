package org.firstinspires.ftc.teamcode.metalBenders.season.decode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;

public class MeepMeepTesting {
    private static AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
//        return AprilTagEnum.RED_TARGET;
    }
    private static StartPositionEnum getStartPosition() {
        return StartPositionEnum.FAR;
//        return StartPositionEnum.NEAR;
    };

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        ColorScheme colorScheme = new ColorSchemeRedDark();
        if(getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            colorScheme = new ColorSchemeBlueDark();
        }
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 3)
                .setDimensions(13,18)
                .setColorScheme(colorScheme)
                .build();


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
        myBot.runAction(new ParallelAction(myBot.getDrive().actionBuilder(startPose)
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
                .build()));

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public enum FieldLocation {
        FIELD_SET_1_LOCATION(
                new Pose2d(-11.5, 27, Math.toRadians(90)),
                new Pose2d(35.5, 27, Math.toRadians(90)),
                new Pose2d(-11.5, -27, Math.toRadians(270)),
                new Pose2d(35.5, -27, Math.toRadians(270))
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
                new Pose2d(-40, 64, Math.toRadians(0)),
                new Pose2d(-40, 64, Math.toRadians(0)),
                new Pose2d(40, -64, Math.toRadians(0)),
                new Pose2d(40, -64, Math.toRadians(0))
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

    public enum ArtifactGroupEnum {
        PRELOAD(null),
        FIELD_SET_1(FieldLocation.FIELD_SET_1_LOCATION),
        FIELD_SET_2(FieldLocation.FIELD_SET_2_LOCATION),
        FIELD_SET_3(FieldLocation.FIELD_SET_3_LOCATION),
        FIELD_SET_4(FieldLocation.FIELD_SET_4_LOCATION);

        private final FieldLocation fieldLocation;

        ArtifactGroupEnum(FieldLocation fieldLocation) {
            this.fieldLocation = fieldLocation;
        }

        private Pose2d getLocation(StartPositionEnum startPositionEnum, AprilTagEnum targetAprilTag) {
            return fieldLocation != null ? fieldLocation.getLocation(startPositionEnum, targetAprilTag) : null;
        }
    }

    public static class LockOnTarget implements Action {

        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            System.out.println("lock on target");
            return false;
        }
    }

    public class Shoot implements Action {
        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            return false;
        }
    }
}