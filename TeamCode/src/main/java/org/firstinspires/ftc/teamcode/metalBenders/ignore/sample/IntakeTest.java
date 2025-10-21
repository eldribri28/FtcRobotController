package org.firstinspires.ftc.teamcode.metalBenders.ignore.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@TeleOp(name = "IntakeTest", group="test")
//@Autonomous(name="TestAuton", group="test")
public class IntakeTest extends LinearOpMode {

    private DcMotorEx intakeMotor;

    private void setupHardware() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
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
            if (gamepad1.left_trigger > 0) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }
            telemetry.update();
        }
    }
}
