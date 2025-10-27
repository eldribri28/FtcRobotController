package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LimelightEngine;

@TeleOp(name = "LimelightTest", group="sample")
public class LimelightTest extends LinearOpMode {

    private HardwareManager hardwareManager;
    private Limelight3A limelight;

    public void runOpMode() {

        initialize();
        waitForStart();

        while (opModeIsActive()) {

            LimelightEngine.getResult(limelight);

        }

    }

    private void initialize() { limelight = hardwareManager.getLimelight(); }

}
