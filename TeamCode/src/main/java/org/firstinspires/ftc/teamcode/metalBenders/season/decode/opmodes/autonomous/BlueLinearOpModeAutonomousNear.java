package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;

@Autonomous(name="BLUE NEAR Linear Autonomous", group="auto-near", preselectTeleOp = "BLUE Linear TeleOp")
public class BlueLinearOpModeAutonomousNear extends AutonomousBaseLinearOpMode {

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }

    @Override
    StartPositionEnum getStartPosition() {
        return StartPositionEnum.NEAR;
    }
}
