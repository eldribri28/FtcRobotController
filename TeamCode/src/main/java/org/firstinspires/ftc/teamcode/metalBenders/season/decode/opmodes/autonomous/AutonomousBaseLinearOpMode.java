package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CHASSIS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;

import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.concurrent.TimeUnit;

public abstract class AutonomousBaseLinearOpMode extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
    private HardwareManager hardwareManager;
    private AprilTagEngine aprilTagEngine;
    private ArtifactColorEnum intakeArtifactColor = ArtifactColorEnum.NONE;
    private ArtifactColorEnum launcherArtifactColor = ArtifactColorEnum.NONE;
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double flywheelRPM = 0;
    private double targetRPM = 0;
    private boolean allowedToLaunch = false;
    private Limelight3A limelight;
    private SparkFunOTOS otos;

    abstract AprilTagEnum getTargetAprilTag();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread aprilTagEngineThread = new Thread(aprilTagEngine);
        try {
            waitForStart();
            resetRuntime();
            boolean isGainAndExposureSet = false;
            while (opModeIsActive()) {
                updateRuntime();
                telemetry.addData("Target name", getTargetAprilTag().name());
                telemetry.addData("Target id", getTargetAprilTag().getId());
                autoLaunch();
                //TODO fill in meat
                telemetry.update();
            }
        } finally {
            aprilTagEngineThread.interrupt();
            aprilTagEngine.teardown();
            telemetry.update();
        }

    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        otos = hardwareManager.getOtos();
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
                "Runtime",
                "%02.0f:%02.0f",
                totalSeconds / 60,
                totalSeconds % 60);
    }

    private void autoLaunch() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if(timedDetection != null && timedDetection.getDetection() != null) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
                if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(targetDetection.ftcPose.bearing)) {
                    telemetry.addData("Turret Angle", (targetDetection.ftcPose.bearing));
                    double setPower = turretBearingPid.calculate(1, targetDetection.ftcPose.bearing);
                    telemetry.addData("Turret Power", setPower);
                    hardwareManager.getTurretMotor().setPower(setPower);

                    if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
                        TURRET_CHASSIS_OFFSET = getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition());
                        double chassisFieldHeading = (targetDetection.robotPose.getOrientation().getYaw() - TURRET_CHASSIS_OFFSET);
                        telemetry.addData("AprilTag Turret Field Heading (deg)", targetDetection.robotPose.getOrientation().getYaw());
                        telemetry.addData("Calculated Chassis Field Heading (deg)", chassisFieldHeading);
                        OTOSCalculator.setCurrentPosition(targetDetection.robotPose.getPosition().x, targetDetection.robotPose.getPosition().y, chassisFieldHeading, otos);
                    }

                } else {
                    hardwareManager.getTurretMotor().setPower(0);
                }
                targetDistance = targetDetection.ftcPose.range;

                flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                telemetry.addData("flywheel RPM", flywheelRPM);

                LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult1(flywheelRPM, targetDistance);
                if (launchResult != null) {
                    setLaunchAngle(launchResult.getLaunchAngle());
                }

                targetRPM = Math.round(((targetDistance / 1.670) * 900.0) + 1750.0);
                hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
                telemetry.addData("target RPM", targetRPM);


                if ( allowedToLaunch ) {
                    if (Math.abs(((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0) - targetRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL) {
                        autoLaunchArtifact();
                    }
                } else {
                    hardwareManager.getLauncherMotor().setVelocity(0);
                }
            } else {
                //stop rotating turret if detection is too old
                hardwareManager.getTurretMotor().setPower(0);
            }
        } else {
            //stop rotating turret if there is no detection
            hardwareManager.getTurretMotor().setPower(0);
        }
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((70.0 - launchAngle) / 35.0));
        if(positionValue >= 0 && positionValue <= 0.8) {
            hardwareManager.getAngleServo().setPosition(positionValue);
        }
        this.launchAngle = launchAngle;
    }

    private void autoLaunchArtifact() {
        if (launcherArtifactColor != ArtifactColorEnum.NONE) {
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_UP);
            sleep(100);
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
            sleep(150);
        }
    }

    /*
    Check to see if the turret is against a limit switch
     */
    private boolean canRotateTurret(double input) {
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()) {
            return false;
        } else if (input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()){
            return false;
        } else {
            return true;
        }
    }

    private double getLimeLightCoordinates() {
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            // Getting numbers from Python
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                double targetFound = pythonOutputs[0];
                double targetBearing = pythonOutputs[1];
                double targetDistance = pythonOutputs[2];
                ArtifactColorEnum targetColor;

                if (pythonOutputs[5] == 1) {
                    targetColor = ArtifactColorEnum.GREEN;
                } else if (pythonOutputs[5] == 2) {
                    targetColor = ArtifactColorEnum.PURPLE;
                } else {
                    targetColor = ArtifactColorEnum.NONE;
                }

                telemetry.addData("LL Target Found", targetFound);
                telemetry.addData("LL Target Bearing (deg)", targetBearing);
                telemetry.addData("LL Target Distance (m)", targetDistance / 1000);
                telemetry.addData("LL Target Color", targetColor);
                telemetry.update();
            }
        }
        return 0;
    }
}
