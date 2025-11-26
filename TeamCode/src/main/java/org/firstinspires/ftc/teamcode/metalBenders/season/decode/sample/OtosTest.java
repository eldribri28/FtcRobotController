package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CHASSIS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CURRENT_ENCODER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSCalculator;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Otos Test", group="sample")
public class OtosTest extends LinearOpMode {

    private HardwareManager hardwareManager;

    private SparkFunOTOS otos;

    private double currentX = 0;
    private double currentY = 0;
    private double currentOtosH = 0;

    public void runOpMode() {

        inititalize();
        setFieldPosition(0, 0, 0);
        waitForStart();

        while (opModeIsActive()) {

            getFieldPosition();

            telemetry.addLine(String.format("Current Position: %6.1f %6.1f %6.1f  (meter, deg)", currentX, currentY, currentOtosH));
            telemetry.update();


        }

    }

    private void inititalize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        //otos = hardwareManager.getOtos();
        //otos.setAngularUnit(AngleUnit.DEGREES);
    }



    private void setFieldPosition(double x, double y, double h) {
        OTOSCalculator.setCurrentPosition(x, y, h, otos);
    }

    private void getFieldPosition() {
        currentX = OTOSCalculator.getCurrentPosition(otos).getXPos();
        currentY = OTOSCalculator.getCurrentPosition(otos).getYPos();
        currentOtosH = OTOSCalculator.getCurrentPosition(otos).getHeading();
        if (currentOtosH > 180) {
            currentOtosH = currentOtosH - 360;
        } else if (currentOtosH <= -180) {
            currentOtosH = currentOtosH + 360;
        }
    }



}
