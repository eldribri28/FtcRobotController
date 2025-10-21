package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

@Autonomous(name="BLUE Linear Autonomous", group="auto")
public class BlueLinearOpModeAutonomous extends AutonomousBaseLinearOpMode {

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }
}
