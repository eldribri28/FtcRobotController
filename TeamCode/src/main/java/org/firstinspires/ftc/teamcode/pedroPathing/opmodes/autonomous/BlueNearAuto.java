package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.AbstractPoseSupplier;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.BlueFarPoseSupplier;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.BlueNearPoseSupplier;

import java.util.List;

import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.FAR_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.LOADING_ZONE_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.MIDDLE_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.NEAR_ARTIFACT_GROUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum.PRELOAD_ARTIFACT_GROUP;

@Autonomous(name="BLUE NEAR Auto", group="auto-near", preselectTeleOp = "BLUE Linear TeleOp")
public class BlueNearAuto extends BaseAuto {

    private static final AbstractPoseSupplier POSE_SUPPLIER = new BlueNearPoseSupplier();

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }

    @Override
    StartPositionEnum getStartPosition() {
        return StartPositionEnum.NEAR;
    }

    @Override
    List<ArtifactGroupEnum> getArtifactGroupExecutionOrder() {
        return List.of(
            PRELOAD_ARTIFACT_GROUP,
            NEAR_ARTIFACT_GROUP,
            MIDDLE_ARTIFACT_GROUP,
            FAR_ARTIFACT_GROUP,
            LOADING_ZONE_ARTIFACT_GROUP
        );
    }

    @Override
    AbstractPoseSupplier getPoseSupplier() {
        return POSE_SUPPLIER;
    }
}
