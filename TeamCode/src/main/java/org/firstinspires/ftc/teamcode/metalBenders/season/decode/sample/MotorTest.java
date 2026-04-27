package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MotorTest", group="sample")
public class MotorTest extends LinearOpMode {

    private DcMotorEx LFmotor;
    private DcMotorEx LRmotor;
    private DcMotorEx RFmotor;
    private DcMotorEx RRmotor;

    private void initHardware() {
        RFmotor = hardwareMap.get(DcMotorEx.class, "RFmotor");
        LFmotor = hardwareMap.get(DcMotorEx.class, "LFmotor");
        RRmotor = hardwareMap.get(DcMotorEx.class, "RRmotor");
        LRmotor = hardwareMap.get(DcMotorEx.class, "LRmotor");
    }

    @Override
    public void runOpMode() {

        initHardware();

        waitForStart();

        while (opModeIsActive()) {

            RFmotor.setPower(0);
            RRmotor.setPower(0);
            LFmotor.setPower(0);
            LRmotor.setPower(0);

            if (gamepad1.a) { RFmotor.setPower(1);}
            if (gamepad1.b) { RRmotor.setPower(1);}
            if (gamepad1.x) { LFmotor.setPower(1);}
            if (gamepad1.y) { LRmotor.setPower(1);}
        }

    }


}