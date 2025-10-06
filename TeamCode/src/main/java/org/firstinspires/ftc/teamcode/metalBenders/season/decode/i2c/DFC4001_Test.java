package org.firstinspires.ftc.teamcode.metalBenders.season.decode.i2c;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DFC4001", group = "Tests")
public class DFC4001_Test extends LinearOpMode
{
    private DFC4001 DFC4001sensor;

    public void runOpMode() throws InterruptedException
    {
        DFC4001sensor = hardwareMap.get(DFC4001.class, "DFC4001sensor");
        DFC4001sensor.startSensor();

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("Status:", DFC4001sensor.getStatus());
            telemetry.addData("Range:", DFC4001sensor.getRange());
            telemetry.update();
            //idle();
            sleep(1000);
        }
    }
}