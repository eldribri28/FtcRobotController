package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = "ServoConf", group="sample")
public class ServoConf extends LinearOpMode  {

    private Servo angleServo;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                angleServo.setPosition(0.3);
            } else {
                angleServo.setPosition(0.8);
            }
        }



    }


    private void initialize() {
        this.angleServo = hardwareMap.get(Servo.class, "angleServo");



    }


}
