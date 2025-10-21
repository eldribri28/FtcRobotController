package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

@Autonomous(name="RED Linear Autonomous", group="auto")
public class RedLinearOpModeAutonomous extends AutonomousBaseLinearOpMode {

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.RED_TARGET;
    }
}
