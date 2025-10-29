package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;

@Autonomous(name="BLUE FAR Linear Autonomous", group="auto-far")
public class BlueLinearOpModeAutonomousFar extends AutonomousBaseLinearOpMode {

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }

    @Override
    StartPositionEnum getStartPosition() {
        return StartPositionEnum.FAR;
    }
}
