package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.AbstractPoseSupplier;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.BlueNearPoseSupplier;

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
    AbstractPoseSupplier getPoseSupplier() {
        return POSE_SUPPLIER;
    }
}