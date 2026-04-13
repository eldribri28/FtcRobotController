package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Test", group="sample")
public class Test extends LinearOpMode {

    private DcMotorEx LFmotor;
    private DcMotorEx LRmotor;
    private DcMotorEx RFmotor;
    private DcMotorEx RRmotor;
    private TouchSensor touch;


    private void initHardware() {
        RFmotor = hardwareMap.get(DcMotorEx.class, "RFmotor");
        LFmotor = hardwareMap.get(DcMotorEx.class, "LFmotor");
        RRmotor = hardwareMap.get(DcMotorEx.class, "RRmotor");
        LRmotor = hardwareMap.get(DcMotorEx.class, "LRmotor");
        RFmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        touch = hardwareMap.get(TouchSensor.class, "touch");
    }

    @Override
    public void runOpMode() {

        initHardware();

        waitForStart();

        while (opModeIsActive()) {

            drive();


            if (touch.isPressed()) {
                telemetry.addData("button", "pressed");
            } else {
                telemetry.addData("button", "not pressed");
            }

            telemetry.update();

        }

    }


    public void drive() {

        double RFpower = 0;
        double RRpower = 0;
        double LFpower = 0;
        double LRpower = 0;

        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            RFpower = RFpower + gamepad1.left_stick_y;
            RRpower = RRpower + gamepad1.left_stick_y;
            LFpower = LFpower + gamepad1.left_stick_y;
            LRpower = LRpower + gamepad1.left_stick_y;
        }
        if (Math.abs(gamepad1.left_stick_x) > 0.1 ) {
            RFpower = RFpower + gamepad1.left_stick_x;
            RRpower = RRpower - gamepad1.left_stick_x;
            LFpower = LFpower - gamepad1.left_stick_x;
            LRpower = LRpower + gamepad1.left_stick_x;
        }
        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            RFpower = RFpower + (0.8 * gamepad1.right_stick_x);
            RRpower = RRpower + (0.8 * gamepad1.right_stick_x);
            LFpower = LFpower - (0.8 * gamepad1.right_stick_x);
            LRpower = LFpower - (0.8 * gamepad1.right_stick_x);
        }

        double max = Math.max(Math.abs(LFpower), Math.abs(RFpower));
        max = Math.max(max, Math.abs(LRpower));
        max = Math.max(max, Math.abs(RRpower));

        if (max > 1.0) {
            LFpower  /= max;
            RFpower /= max;
            LRpower   /= max;
            RRpower  /= max;
        }

        RFmotor.setPower(RFpower);
        RRmotor.setPower(RRpower);
        LFmotor.setPower(LFpower);
        LRmotor.setPower(LRpower);

    }

}