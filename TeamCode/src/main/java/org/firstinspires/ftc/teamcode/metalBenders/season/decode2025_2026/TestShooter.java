package org.firstinspires.ftc.teamcode.metalBenders.season.decode2025_2026;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TestShooter")
public class TestShooter extends LinearOpMode {
    private DcMotorEx shooterMotor;

    private void setupHardware() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Raise Arm");;
    }

    @Override
    public void runOpMode() {
        setupHardware();
        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {
            if(gamepad1.right_trigger > 0) {
                shooterMotor.setPower(1);
            } else {
                shooterMotor.setPower(0);
            }
            if(gamepad1.left_trigger > 0) {
                shooterMotor.setPower(-1);
            } else {
                shooterMotor.setPower(0);
            }
            telemetry.addData("motor power", shooterMotor.getPower());
            telemetry.addData("motor current (AMPS)", shooterMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
