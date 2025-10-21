package org.firstinspires.ftc.teamcode.metalBenders.ignore.sample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name = "ServoAngle")
public class ServoAngle extends LinearOpMode {

    private Servo angleServo;
    private Servo launchServo;

    private Gamepad gamepad;

    public void runOpMode() {

        angleServo = hardwareMap.get(Servo.class, "angleServo");
        launchServo = hardwareMap.get(Servo.class, "launchServo");

        angleServo.setDirection(Servo.Direction.REVERSE);

        this.gamepad = gamepad1;

        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {

            //Base Angle 58deg
            //Extended Angle 15deg

            //double desiredDegrees = 20; // Example: 45 degrees
            //double positionValue = desiredDegrees / 45;
            //angleServo.setPosition(positionValue);

            if(gamepad.a == true) {
                launchServo.setPosition(0);
            } else {
                launchServo.setPosition(0.7);
            }



        }

    }





}
