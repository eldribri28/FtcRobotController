package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AutonStateEnum;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
    private boolean launcherEnabled = false;
    private boolean atTargetPosition = true;
    private double stateTimestamp = getRuntime();
    private Limelight3A limelight;
    private SparkFunOTOS otos;
    abstract AprilTagEnum getTargetAprilTag();
    AutonStateEnum state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread aprilTagEngineThread = new Thread(aprilTagEngine);
        try {
            waitForStart();
            resetRuntime();

            while (opModeIsActive()) {
                updateRuntime();
                telemetry.addData("Target name", getTargetAprilTag().name());
                telemetry.addData("Target id", getTargetAprilTag().getId());
                switch (state) {
                    case WAIT_LAUNCH_ARTIFACT_1:
                        launcherEnabled = true;
                        if (launcherArtifactColor != ArtifactColorEnum.NONE || getRuntime() - stateTimestamp < 5) {
                            allowedToLaunch = true;
                            autoLaunch();
                        } else {
                            allowedToLaunch = false;
                            state = AutonStateEnum.WAIT_INTAKE_LOAD_ARTIFACT_2;
                            stateTimestamp = getRuntime();
                        }
                        break;
                    case WAIT_INTAKE_LOAD_ARTIFACT_2:
                        if (launcherArtifactColor == ArtifactColorEnum.NONE || getRuntime() - stateTimestamp < 5) {
                            startIntake();
                        } else {
                            stopIntake();
                            state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_2;
                            stateTimestamp = getRuntime();
                        }
                        break;
                    case WAIT_LAUNCH_ARTIFACT_2:
                        launcherEnabled = true;
                        if (launcherArtifactColor != ArtifactColorEnum.NONE || getRuntime() - stateTimestamp < 5) {
                            allowedToLaunch = true;
                            autoLaunch();
                        } else {
                            allowedToLaunch = false;
                            state = AutonStateEnum.WAIT_INTAKE_LOAD_ARTIFACT_2;
                            stateTimestamp = getRuntime();
                        }
                        break;
                    case WAIT_INTAKE_LOAD_ARTIFACT_3:
                        if (launcherArtifactColor == ArtifactColorEnum.NONE || getRuntime() - stateTimestamp < 5) {
                            startIntake();
                        } else {
                            stopIntake();
                            state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_3;
                            stateTimestamp = getRuntime();
                        }
                        break;
                    case WAIT_LAUNCH_ARTIFACT_3:
                        launcherEnabled = true;
                        if (launcherArtifactColor != ArtifactColorEnum.NONE || getRuntime() - stateTimestamp < 5) {
                            allowedToLaunch = true;
                            autoLaunch();
                        } else {
                            allowedToLaunch = false;
                            state = AutonStateEnum.WAIT_DRIVE_FROM_LAUNCH_ZONE;
                            stateTimestamp = getRuntime();
                            atTargetPosition = false;
                        }
                        break;
                    case WAIT_DRIVE_FROM_LAUNCH_ZONE:
                        launcherEnabled = false;
                        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET && !atTargetPosition) {

                        } else {
                            state = AutonStateEnum.FINISHED;
                            stateTimestamp = getRuntime();
                        }
                    case FINISHED:
                        break;
                    default:
                        stateTimestamp = getRuntime();
                        state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_1;
                }
                timeIsUpMove();
                refreshTelemetry();
            }
        } finally {
            aprilTagEngineThread.interrupt();
            aprilTagEngine.teardown();
            refreshTelemetry();
        }

    }

    private void refreshTelemetry() {
        telemetry.addData("Current State", state);
        telemetry.addData("Time in State (s)", (getRuntime() - stateTimestamp));
        telemetry.addData("Target Flywheel RPM", targetRPM);
        telemetry.addData("Actual Flywheel RPM", flywheelRPM);
        telemetry.addData("Launcher Angle", launchAngle);
        telemetry.update();
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        otos = hardwareManager.getOtos();
        state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_1;
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
                "Runtime",
                "%02.0f:%02.0f",
                totalSeconds / 60,
                totalSeconds % 60);
    }

    private boolean timeIsUpMove() {
        double totalSeconds = getRuntime();
        if (totalSeconds > 25 && state != AutonStateEnum.FINISHED) {
            state = AutonStateEnum.WAIT_DRIVE_FROM_LAUNCH_ZONE;
            atTargetPosition = false;
            return true;
        } else {
            return false;
        }
    }

    private void autoLaunch() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if(timedDetection != null && timedDetection.getDetection() != null) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {

                /*
                Aquire target lock
                 */
                if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(targetDetection.ftcPose.bearing)) {
                    telemetry.addData("Turret Angle", (targetDetection.ftcPose.bearing));
                    double setPower = turretBearingPid.calculate(1, targetDetection.ftcPose.bearing);
                    telemetry.addData("Turret Power", setPower);
                    hardwareManager.getTurretMotor().setPower(setPower);
                } else {
                    hardwareManager.getTurretMotor().setPower(0);
                }

                /*
                Generate launch system parameters and generate firing solution
                 */
                targetDistance = targetDetection.ftcPose.range;
                flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                targetRPM = Math.round(((targetDistance / 1.670) * 900.0) + 1750.0);
                LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult1(flywheelRPM, targetDistance);

                if (launcherEnabled) {
                    if (launchResult != null) {
                        setLaunchAngle(launchResult.getLaunchAngle());
                    }
                    hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
                    if (Math.abs(((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0) - targetRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL && allowedToLaunch) {
                        autoLaunchArtifact();
                    }
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

    private void startIntake() {
        hardwareManager.getIntakeMotor().setPower(1);
    }
    private void stopIntake() {
        hardwareManager.getIntakeMotor().setPower(1);
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
