package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.AbstractPoseSupplier;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.RedFarPoseSupplier;

@Autonomous(name="RED FAR Auto", group="auto-far", preselectTeleOp = "RED Linear TeleOp")
public class RedFarAuto extends BaseAuto {

    private static final AbstractPoseSupplier POSE_SUPPLIER = new RedFarPoseSupplier();

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.RED_TARGET;
    }

    @Override
    StartPositionEnum getStartPosition() {
        return StartPositionEnum.FAR;
    }

    @Override
    AbstractPoseSupplier getPoseSupplier() {
        return POSE_SUPPLIER;
    }
}