package org.firstinspires.ftc.teamcode.metalBenders.ignore.sample;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@TeleOp(name = "TurretTestPid")
public class TurretTestPid extends LinearOpMode {
    private Gamepad gamepad;
    private DcMotorEx turretMotor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final PIDController bearingPid = new PIDController(0.05, 0.005, 0.05);

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {
            if(gamepad.left_trigger > 0) {
                turretMotor.setPower(-0.5);
            } else if (gamepad.right_trigger > 0) {
                turretMotor.setPower(0.5);
            } else {
                turretMotor.setPower(0);
            }
            telemetryAprilTag();
            telemetry.update();
        }
        visionPortal.close();
    }

    private void initAprilTagProcessor() {
        aprilTagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.364794005).build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "TurretCam"));
        builder.addProcessor(aprilTagProcessor);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(640,480));


        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            turretMotor.setPower(-bearingPid.calculate(1, detection.ftcPose.bearing));


            LaunchResult launchResults = LaunchCalculator.calculatePreferredLaunchResult(detection.ftcPose.range, 3500);

            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addData("Calculated Launch Angle (deg)", Math.toDegrees(launchResults.getLaunchAngle()));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    private void initialize() {
        this.gamepad = gamepad1;
        initializeHardware();
    }

    private void initializeHardware() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(RUN_USING_ENCODER);
        initAprilTagProcessor();
    }
}
