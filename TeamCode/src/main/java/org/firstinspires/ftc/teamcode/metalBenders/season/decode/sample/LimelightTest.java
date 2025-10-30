package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;

@TeleOp(name = "LimelightTest", group="sample")
public class LimelightTest extends LinearOpMode {

    private HardwareManager hardwareManager;
    private Limelight3A limelight;

    public void runOpMode() {

        initialize();
        waitForStart();

        while (opModeIsActive()) {


        }

    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        limelight = hardwareManager.getLimelight();
    }

}
