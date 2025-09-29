package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TestShooter")
public class TestShooter extends LinearOpMode {
    private DcMotorEx launcherMotor;

    private void setupHardware() {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
    }

    @Override
    public void runOpMode() {
        setupHardware();
        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {
            //if(gamepad1.right_trigger > 0) {
            //    launcherMotor.setPower(-0.75);
            //} else {
            //    launcherMotor.setPower(0);
            //}
            //if(gamepad1.left_trigger > 0) {
            //    launcherMotor.setPower(-1);
            //} else {
            //    launcherMotor.setPower(0);
            //}
            double targetRPM = 3700;
            if(gamepad1.right_trigger > 0) {
                launcherMotor.setVelocity(-( targetRPM / 60 ) * 28);
            } else {
                launcherMotor.setVelocity(0);
            }

            double flywheelRPM = (launcherMotor.getVelocity()/28) * 60;
            telemetry.addData("shooter motor1 power", launcherMotor.getPower());
            telemetry.addData("shooter motor1 current (AMPS)", launcherMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("shooter motor1 velocity", flywheelRPM);
            telemetry.update();
        }
    }
}
