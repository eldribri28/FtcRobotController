package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.AbstractPoseSupplier;
import org.firstinspires.ftc.teamcode.pedroPathing.pose.BlueFarPoseSupplier;

@Autonomous(name="BLUE FAR Auto", group="auto-far", preselectTeleOp = "BLUE Linear TeleOp")
public class BlueFarAuto extends BaseAuto {

    private static final AbstractPoseSupplier POSE_SUPPLIER = new BlueFarPoseSupplier();

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
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
