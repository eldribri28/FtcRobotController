package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoConf", group="sample")
public class ServoConf extends LinearOpMode  {

    private Servo launchServo;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                launchServo.setPosition(0.75);
            } else {
                launchServo.setPosition(0.4);
            }
        }



    }


    private void initialize() {
        this.launchServo = hardwareMap.get(Servo.class, "launchServo");



    }


}
