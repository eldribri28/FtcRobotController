package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Motor Encoder Test", group="sample")
public class motorEncoderTest extends LinearOpMode {

    private HardwareManager hardwareManager;

    public void runOpMode() {

        initialize();
        waitForStart();

        while (opModeIsActive()) {
            hardwareManager.getLeftFrontMotor().setVelocity(100);
            hardwareManager.getLeftRearMotor().setVelocity(300);
            hardwareManager.getRightFrontMotor().setVelocity(500);
            hardwareManager.getRightRearMotor().setVelocity(700);

            telemetry.addData("Left Front Encoder (100)", hardwareManager.getLeftFrontMotor().getVelocity());
            telemetry.addData("Left Rear Encoder (300)", hardwareManager.getLeftRearMotor().getVelocity());
            telemetry.addData("Right Front Encoder (500)", hardwareManager.getRightFrontMotor().getVelocity());
            telemetry.addData("Right Rear Encoder (700)", hardwareManager.getRightRearMotor().getVelocity());
            telemetry.update();

        }


    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);

    }

}
