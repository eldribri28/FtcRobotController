package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.CAMERA_EXPOSURE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.CAMERA_GAIN;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name="TestWebcam", group="Sample")
public class TestWebcam extends LinearOpMode {

    private VisionPortal visionPortal;
    ExposureControl myExposureControl;  // declare exposure control object
    GainControl myGainControl;
    long minExp;
    long maxExp;
    long curExp;            // exposure is duration, in time units specified

    int minGain;
    int maxGain;
    int curGain;
    boolean wasSetGainSuccessful;   // returned from setGain()

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "TurretCam");
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcamName);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(640, 480));
        visionPortal = builder.build();

        waitForCameraStreamToStart();

        myExposureControl = visionPortal.getCameraControl(ExposureControl.class);
        myGainControl = visionPortal.getCameraControl(GainControl.class);

        // Display exposure features and settings of this webcam.
        checkExposureFeatures();

        sleep(100);
        myExposureControl.setMode(ExposureControl.Mode.Manual);
        sleep(100);
        myExposureControl.setAePriority(false);
        sleep(100);
        myExposureControl.setExposure(7, TimeUnit.MILLISECONDS);
        sleep(100);
        myGainControl.setGain(5);
        sleep(100);

        // Retrieve from webcam its current exposure and gain values.
        curExp = myExposureControl.getExposure(TimeUnit.MILLISECONDS);
        curGain = myGainControl.getGain();

        // Display mode and starting values to user.
        telemetry.addLine("\nTouch Start arrow to control webcam Exposure and Gain");
        telemetry.addData("\nCurrent exposure mode", myExposureControl.getMode());
        telemetry.addData("Current exposure value", curExp);
        telemetry.addData("Current gain value", curGain);
        telemetry.update();


        waitForStart();

        telemetry.addLine("\nTouch Start arrow to control webcam Exposure and Gain");

        // Get webcam exposure limits.
        minExp = myExposureControl.getMinExposure(TimeUnit.MILLISECONDS);
        maxExp = myExposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        // Get webcam gain limits.
        minGain = myGainControl.getMinGain();
        maxGain = myGainControl.getMaxGain();

        while (!opModeIsActive()) {
            setAePriority();
            telemetry.addData("Exposure", "Min:%d, Max:%d, Current:%d", minExp, maxExp, curExp);
            telemetry.addData("Gain", "Min:%d, Max:%d, Current:%d", minGain, maxGain, curGain);
            telemetry.addData("Gain change successful?", wasSetGainSuccessful);
            telemetry.addData("Current exposure mode", myExposureControl.getMode());


            telemetry.update();
        }



        while (opModeIsActive()) {

            // Manually adjust the webcam exposure and gain variables.
            float changeExp = -gamepad1.left_stick_y;
            float changeGain = -gamepad1.right_stick_y;

            int changeExpInt = (int) (changeExp*5);
            int changeGainInt = (int) (changeGain*5);

            curExp += changeExpInt;
            curGain += changeGainInt;

            // Ensure inputs are within webcam limits, if provided.
            curExp = Math.max(curExp, minExp);
            curExp = Math.min(curExp, maxExp);
            curGain = Math.max(curGain, minGain);
            curGain = Math.min(curGain, maxGain);

            // Update the webcam's settings.
            myExposureControl.setExposure(curExp, TimeUnit.MILLISECONDS);
            wasSetGainSuccessful = myGainControl.setGain(curGain);

            setAePriority();

            telemetry.addLine("\nExposure: left stick Y; Gain: right stick Y");
            telemetry.addData("Exposure", "Min:%d, Max:%d, Current:%d", minExp, maxExp, curExp);
            telemetry.addData("Gain", "Min:%d, Max:%d, Current:%d", minGain, maxGain, curGain);
            telemetry.addData("Gain change successful?", wasSetGainSuccessful);
            telemetry.addData("Current exposure mode", myExposureControl.getMode());

            telemetry.addLine("\nAutoExposure Priority: green A ON; red B OFF");
            telemetry.addData("AutoExposure Priority?", myExposureControl.getAePriority());
            telemetry.update();

            sleep(100);

        }
        visionPortal.close();
    }

    // Display the exposure features and modes supported by this webcam.
    private void checkExposureFeatures() {

        while (!gamepad1.y && !isStopRequested()) {
            telemetry.addLine("**Exposure settings of this webcam:");
            telemetry.addData("Exposure control supported?", myExposureControl.isExposureSupported());
            telemetry.addData("Autoexposure priority?", myExposureControl.getAePriority());

            telemetry.addLine("\n**Exposure Modes supported by this webcam:");
            telemetry.addData("AperturePriority", myExposureControl.isModeSupported(ExposureControl.Mode.AperturePriority));
            telemetry.addData("Auto", myExposureControl.isModeSupported(ExposureControl.Mode.Auto));
            telemetry.addData("ContinuousAuto", myExposureControl.isModeSupported(ExposureControl.Mode.ContinuousAuto));
            telemetry.addData("Manual", myExposureControl.isModeSupported(ExposureControl.Mode.Manual));
            telemetry.addData("ShutterPriority", myExposureControl.isModeSupported(ExposureControl.Mode.ShutterPriority));
            telemetry.addData("Unknown", myExposureControl.isModeSupported(ExposureControl.Mode.Unknown));
            telemetry.addLine("*** PRESS Y TO CONTINUE ***");
            telemetry.update();
        }

    }   // end method checkExposureFeatures()

    private void waitForCameraStreamToStart() {
        while (!Thread.currentThread().isInterrupted()
                && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e){}
        }
    }

    private void setAePriority() {
        // Manually set Auto-Exposure Priority.
        if (gamepad1.a) {                           // turn on with green A
            myExposureControl.setAePriority(true);
        } else if (gamepad1.b) {                    // turn off with red B
            myExposureControl.setAePriority(false);
        }
    }
}