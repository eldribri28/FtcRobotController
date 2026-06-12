package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.AbstractPoseSupplier;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.RedNearPoseSupplier;

@Autonomous(name="RED NEAR Auto", group="auto-near", preselectTeleOp = "RED Linear TeleOp")
public class RedNearAuto extends BaseAuto {

    private static final AbstractPoseSupplier POSE_SUPPLIER = new RedNearPoseSupplier();

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.RED_TARGET;
    }

    @Override
    StartPositionEnum getStartPosition() {
        return StartPositionEnum.NEAR;
    }

    @Override
    AbstractPoseSupplier getPoseSupplier() {
        return POSE_SUPPLIER;
    }
}