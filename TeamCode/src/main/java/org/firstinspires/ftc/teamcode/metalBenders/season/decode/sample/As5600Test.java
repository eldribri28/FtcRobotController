package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_ENCODER_DEGREES_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_ENCODER_ROBOT_POSE_ZERO;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_TICKS_PER_DEGREE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.i2c.AS5600;

@Disabled
@TeleOp(name="As5600Test", group="Sample")
public class As5600Test extends LinearOpMode {

    private HardwareManager hardwareManager;
    private double turretMotorEncoderZero = 0;

    @Override
    public void runOpMode() {

        initialize();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Encoder Angle (deg)", hardwareManager.getTurretEncoder().getAngleDeg(true));
            telemetry.addData("Turret Angle (deg)", getTurretAngleFromEncoder());
            telemetry.update();
            sleep(20);

        }

    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        setTurretEncoderToMotorEncoderOffset();
    }

    private void setTurretEncoderToMotorEncoderOffset() {
        long currentTurretMotorEncoderValue = hardwareManager.getTurretMotor().getCurrentPosition();
        double currentTurretEncoderRawPosition = hardwareManager.getTurretEncoder().getAngleDeg(true);
        double currentTurretEncoderAdjustedPosition = AngleUnit.normalizeDegrees(TURRET_ENCODER_ROBOT_POSE_ZERO - currentTurretEncoderRawPosition);
        double currentTurretActualPosition = AngleUnit.normalizeDegrees(currentTurretEncoderAdjustedPosition / TURRET_ENCODER_DEGREES_TO_ACTUAL);
        turretMotorEncoderZero = currentTurretMotorEncoderValue - (long)(currentTurretActualPosition * TURRET_TICKS_PER_DEGREE);
    }

    private double getTurretAngleFromEncoder() {
        long currentTurretMotorEncoderValue = hardwareManager.getTurretMotor().getCurrentPosition();
        return (currentTurretMotorEncoderValue - turretMotorEncoderZero) / TURRET_TICKS_PER_DEGREE;
    }

}
