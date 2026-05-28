package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.APRIL_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.NO_LAUNCH_SOLUTION;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.NO_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.VIABLE_LAUNCH_SOLUTION;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_NO_POWER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_POWER_IN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_POWER_OUT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCHER_MOTOR_IDLE_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_GATE_OPEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_FAR_LAUNCH_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_LAUNCH_MOTOR_VELOCITY_START;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_NEAR_LAUNCH_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCH_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MIN_LAUNCH_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TARGET_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_DRIVE_VELOCITY_METER_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_LAST_TIMESTAMP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_TARGET_CLOSE_RATE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_TARGET_YAW_RATE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CHASSIS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_H;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.calculateTransitTime;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.calculateVelocity;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.getAngleForFlywheel;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.ShotCalculator.calculateLeadAngle;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.ShotCalculator.updateTargetDiff;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.IndicatorLedEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
//import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.ColorManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.ShotCalculator;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;


public abstract class TeleOpBaseLinearOpMode extends LinearOpMode {
    private HardwareManager hardwareManager;
    private ArtifactMotifEnum artifactMotifEnum = ArtifactMotifEnum.UNKNOWN;
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double flywheelRPM = 0;
    private double targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
    private double launchVelocity = 0;
    private double launchTOF = 0;
    private double launchLeadAngle = 0;
    private double targetBearing = 0;
    private double targetYaw = 0;
    private double turretError = 0;
    private double botHeadingRad = 0;
    private double botHeadingDeg = 0;
    private double imuTargetHeading = 0;
    private long detectionTimestamp = System.currentTimeMillis();
    private boolean launchSolution = false;
    private AprilTagEngine aprilTagEngine;
    private boolean isManualLaunchOverrideActive = false;
    private double manualLaunchVelocity = MANUAL_LAUNCH_MOTOR_VELOCITY_START;
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    //private ColorManager colorManager;
    abstract AprilTagEnum getTargetAprilTag();
    //private Thread colorManagerThread;
    private Thread aprilTagEngineThread;
    boolean firing = false;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            initialize();
            hardwareManager.getAngleServo().setPosition(0);
            waitForStart();
            stopLaunchArtifact();
            resetRuntime();
            aprilTagEngineThread.start();
            hardwareManager.postStartInitialization();
            while (opModeIsActive()) {
                updateTelemetry();
                resetIMU();
                drive();
                intakeOrRejectArtifact();
                clearArtifactFromLaunch();
                setArtifactMotifEnum();
                setManualLaunchOverride(aprilTagEngineThread);
                launch();
                moveIntake();
                telemetry.update();
            }
        } finally {
            scheduler.shutdownNow();
            if(aprilTagEngineThread != null) {
                aprilTagEngineThread.interrupt();
                aprilTagEngine.teardown();
            }
            //if(colorManagerThread != null) {
            //    colorManagerThread.interrupt();
            //}
            telemetry.update();
        }
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        aprilTagEngineThread = new Thread(aprilTagEngine);
        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.RED.getLedValue());
        hardwareManager.getIntakeServo().setPosition(INTAKE_DOWN);
        //colorManager = new ColorManager(hardwareManager);
        //colorManagerThread = new Thread(colorManager);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
                "Runtime",
                "%02.0f:%02.0f",
                totalSeconds / 60,
                totalSeconds % 60);
    }

    private void updateTelemetry() {
        updateRuntime();
        //telemetry.addData("Manual Launch Controls Active", isManualLaunchOverrideActive);
        telemetry.addData("Target name", getTargetAprilTag().name());
        telemetry.addData("Target distance", targetDistance);
        telemetry.addData("Launch Angle", launchAngle);
        telemetry.addData("Launch Velocity", launchVelocity);
        telemetry.addData("Turret Limit Switch Left Pressed", hardwareManager.getLimitSwitchLeft().isPressed());
        telemetry.addData("Turret Limit Switch Right Pressed", hardwareManager.getLimitSwitchRight().isPressed());
        telemetry.addData("Turret Angle: (deg)", getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition()));
        telemetry.addData("Target Close Rate (M/S)", ROBOT_TARGET_CLOSE_RATE);
        telemetry.addData("Target Yaw Rate (Deg/S)", ROBOT_TARGET_YAW_RATE);
        telemetry.addData("Target Last Timestamp (ms)", ROBOT_LAST_TIMESTAMP);

        aprilTagEngine.getTelemetry().forEach((k, v) -> telemetry.addData(k, v));
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x * 1.0;
        updateBotHeading();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeadingRad) - y * Math.sin(-botHeadingRad);
        double rotY = x * Math.sin(-botHeadingRad) + y * Math.cos(-botHeadingRad);

        //rotX = rotX * 1.1;  // Counteract imperfect strafing

        //calculate power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftRearPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightRearPower = (rotY + rotX - rx) / denominator;

        double driverSelectedMultiplier = 1.0;
        if(gamepad1.rightBumperWasPressed()) {
            driverSelectedMultiplier = 0.8;
        } else if (gamepad1.rightTriggerWasPressed()) {
            driverSelectedMultiplier = 0.5;
        }
        double leftFrontVelocity = DRIVE_MOTOR_POWER * leftFrontPower * driverSelectedMultiplier;
        double rightFrontVelocity = DRIVE_MOTOR_POWER * rightFrontPower * driverSelectedMultiplier;
        double leftRearVelocity = DRIVE_MOTOR_POWER * leftRearPower * driverSelectedMultiplier;
        double rightRearVelocity = DRIVE_MOTOR_POWER * rightRearPower * driverSelectedMultiplier;

        if(leftFrontVelocity == 0) {
            hardwareManager.getLeftFrontMotor().setPower(0);
        } else {
            hardwareManager.getLeftFrontMotor().setPower(leftFrontVelocity);
        }
        if(rightFrontVelocity == 0) {
            hardwareManager.getRightFrontMotor().setPower(0);
        } else {
            hardwareManager.getRightFrontMotor().setPower(rightFrontVelocity);
        }
        if(leftRearVelocity == 0) {
            hardwareManager.getLeftRearMotor().setPower(0);
        } else {
            hardwareManager.getLeftRearMotor().setPower(leftRearVelocity);
        }
        if(rightRearVelocity == 0) {
            hardwareManager.getRightRearMotor().setPower(0);
        } else {
            hardwareManager.getRightRearMotor().setPower(rightRearVelocity);
        }

    }

    private void updateBotHeading() {
        YawPitchRollAngles orientation = hardwareManager.getImu().getRobotYawPitchRollAngles();
        botHeadingRad = orientation.getYaw(AngleUnit.RADIANS);
        botHeadingDeg = orientation.getYaw(AngleUnit.DEGREES);
    }

    private void resetIMU() {
        if (hardwareManager.getGamepad1().dpad_right) {
            hardwareManager.getImu().resetYaw();
        }
    }

    private void intakeOrRejectArtifact() {
        if (hardwareManager.getGamepad1().left_bumper) {
            hardwareManager.getIntakeMotor().setPower(INTAKE_POWER_OUT);
        } else if (hardwareManager.getGamepad1().left_trigger_pressed) {
            hardwareManager.getIntakeMotor().setPower(INTAKE_POWER_IN);
        } else {
            hardwareManager.getIntakeMotor().setPower(INTAKE_NO_POWER);
        }
    }

    private void setManualLaunchOverride(Thread aprilTagEngineThread) {
        if(hardwareManager.getGamepad2().dpad_left && hardwareManager.getGamepad2().x) {
            isManualLaunchOverrideActive = true;
            aprilTagEngineThread.interrupt();
            if(getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
                hardwareManager.getTurretMotor().setPower(-0.8);
            } else {
                hardwareManager.getTurretMotor().setPower(0.8);
            }
        } else if (hardwareManager.getGamepad1().dpad_left && hardwareManager.getGamepad1().x) {
            isManualLaunchOverrideActive = false;
            aprilTagEngineThread.start();
        }
    }

    private void launch() {
        if(isManualLaunchOverrideActive) {
            manualLaunch();
        } else {
            autoLaunch();
        }
    }

    private void moveIntake() {
        if (hardwareManager.getGamepad1().dpad_up) {
            hardwareManager.getIntakeServo().setPosition(INTAKE_UP);
        } else if (hardwareManager.getGamepad1().dpad_down) {
            hardwareManager.getIntakeServo().setPosition(INTAKE_DOWN);
        }
    }


    private void manualLaunch() {
        hardwareManager.getLauncherMotor().setVelocity(manualLaunchVelocity);

        if(hardwareManager.getGamepad2().right_trigger_pressed) {
            autoLaunchArtifact();
        }
    }

    private boolean canRotateTurret(double input) {
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()) {
            return false;
        } else if (input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()){
            return false;
        } else {
            return true;
        }
    }

    private void setArtifactMotifEnum() {
        //get the motif if unknown, may continue to be unknown motif tag is seen
        if(artifactMotifEnum == ArtifactMotifEnum.UNKNOWN) {
            artifactMotifEnum = aprilTagEngine.getArtifactMotif();
        }
    }

    /*
    Target lead calculations to move while shooting.  Assumes infinite acceleration of chassis and no obstacles.
     */
    private double targetLeadCalculation() {

        double transitTime = calculateTransitTime(targetDistance, calculateVelocity(flywheelRPM), launchAngle);
        double angleChange = 0;

        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET && flywheelRPM > 0) {

            double deltaY = targetDistance - (hardwareManager.getGamepad1().left_stick_y * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);
            double deltaX = 0 - (hardwareManager.getGamepad1().left_stick_x * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);

            angleChange = (Math.toDegrees(Math.atan2(deltaY, deltaX)) - 90) * 0.5;

        } else if (getTargetAprilTag() == AprilTagEnum.RED_TARGET && flywheelRPM > 0) {

            double deltaY = targetDistance - (hardwareManager.getGamepad1().left_stick_y * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);
            double deltaX = 0 - (hardwareManager.getGamepad1().left_stick_x * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);

            angleChange = (-Math.toDegrees(Math.atan2(deltaY, deltaX)) + 90) * 0.5;

        }

        telemetry.addData("Turret Lead (deg)", angleChange);
        return angleChange;

    }

    private void autoLaunch() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if(timedDetection != null && timedDetection.getDetection() != null) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            detectionTimestamp = System.currentTimeMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
                setLedStates(APRIL_TAG_DETECTED);

                targetDistance = targetDetection.ftcPose.range;
                targetBearing = targetDetection.ftcPose.bearing;
                targetYaw = targetDetection.ftcPose.yaw;
                ROBOT_FIELD_X = targetDetection.robotPose.getPosition().x;
                ROBOT_FIELD_Y = targetDetection.robotPose.getPosition().y;
                ROBOT_FIELD_H = targetDetection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                double turretError = calculateBearingToGoal(ROBOT_FIELD_X, ROBOT_FIELD_Y, ROBOT_FIELD_H);
                if (targetDistance > 2.7) {
                    turretError -= 4;
                }

                telemetry.addData("Robot X", ROBOT_FIELD_X);
                telemetry.addData("RobotY", ROBOT_FIELD_Y);
                telemetry.addData("RobotH", ROBOT_FIELD_H);
                telemetry.addData("Target Bearing", targetBearing);

                flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                telemetry.addData("flywheel RPM", flywheelRPM);

                updateTargetDiff(targetYaw, targetDistance);

                LaunchCalculator.LaunchResult launchResult = LaunchCalculator.getLaunchData(LAUNCH_HEIGHT, TARGET_HEIGHT, targetDistance, flywheelRPM, ROBOT_TARGET_CLOSE_RATE);
                launchVelocity = launchResult.getLaunchVelocity();
                launchAngle = launchResult.getLaunchAngle();
                launchTOF = launchResult.getTOF();

                setLaunchAngle(launchAngle);

                if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(targetBearing) && targetDetection.id == getTargetAprilTag().getId()) {
                    telemetry.addData("Turret Angle", (targetBearing));
                    telemetry.addData("Target ID", targetDetection.id);


                    double setPower = turretBearingPid.calculate(0, turretError) * 0.8;
                    if (!canRotateTurret(setPower)) {
                        setPower = 0;
                    }
                    telemetry.addData("Turret Power", setPower);
                    telemetry.addData("Turret Error", turretError);
                    hardwareManager.getTurretMotor().setPower(setPower);

                } else {
                    setNoTagDetected();
                    hardwareManager.getTurretMotor().setPower(0);
                }

                if (launchVelocity > 0 || launchAngle > 0) {
                    launchSolution = true;
                    if (readyToShoot()) {
                        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.GREEN.getLedValue());
                    } else {
                        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.YELLOW.getLedValue());
                    }

                } else {
                    launchSolution = false;
                    hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.ORANGE.getLedValue());
                }

                targetRPM = Math.round(launchResult.getFlywheelRpm());
                //if (hardwareManager.getGamepad1().right_trigger_pressed) {
                //    if (!firing && readyToShoot()) {
                //        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.GREEN.getLedValue());
                //        firing = true;
                //        autoLaunchArtifact();
                //    }
                //} else {
                //    //targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
                //    firing = false;
                //    stopLaunchArtifact();
                //}
                if (!hardwareManager.getGamepad1().a) {
                    stopLaunchArtifact();
                }
                telemetry.addData("target RPM", targetRPM);
            } else {
                setNoTagDetected();
            }
        } else {
            setNoTagDetected();

        }

        if (System.currentTimeMillis() - detectionTimestamp < 200) {
            turretError = imuTargetHeading - botHeadingDeg;
            while (turretError > 180) turretError -= 360;
            while (turretError < -180) turretError += 360;

        } else {
            hardwareManager.getTurretMotor().setPower(0);
        }

        hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
    }

    private double calculateBearingToGoal(double robotX, double robotY, double robotYaw) {

        double goalX = 0;
        double goalY = 0;

        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            goalX = -1.8288;
            goalY = -1.8288;
        } else {
            goalX = -1.8288;
            goalY = 1.8288;
        }

        // Robot field angle to goal
        double absoluteAngleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

        telemetry.addData("Angle to Target", absoluteAngleToGoal);
        // Calculate turret angle to goal
        double relativeAngleToGoal = absoluteAngleToGoal - robotYaw - 90;

        // Normalize
        while (relativeAngleToGoal > 180) relativeAngleToGoal -= 360;
        while (relativeAngleToGoal < -180) relativeAngleToGoal += 360;

        return relativeAngleToGoal * 0.85;

    }

    private void setNoTagDetected() {
        hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.RED.getLedValue());
        //targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
        if (!hardwareManager.getGamepad1().a) {
            hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_CLOSE);
        }
        setLedStates(NO_TAG_DETECTED);
    }

    public boolean readyToShoot() {
        return isTurretAngleWithinThreshold()
                && isLaunchMotorVelocityWithinThreshold()
                && launchSolution;
    }

    private boolean isTurretAngleWithinThreshold() {
        double bearing = Math.abs(turretError);
        if(targetDistance > 2.0) {
            return bearing <= 0.3;
        }
        return bearing <= 0.75;
    }

    private boolean isLaunchMotorVelocityWithinThreshold() {
        double currentRPM = ((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0);
        return Math.abs(targetRPM - currentRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
    }

    private void setLedStates(LedStateEnum ledStateEnum) {
        if(ledStateEnum == APRIL_TAG_DETECTED) {
            hardwareManager.getRedLed().on();
            hardwareManager.getGreenLed().off();
        } else if (ledStateEnum == VIABLE_LAUNCH_SOLUTION) {
            hardwareManager.getRedLed().off();
            hardwareManager.getGreenLed().on();
        } else {
            hardwareManager.getRedLed().off();
            hardwareManager.getGreenLed().off();
        }
    }

    private void autoLaunchArtifact() {
        hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_OPEN);
        hardwareManager.getIntakeMotor().setPower(INTAKE_POWER_IN);
    }

    private void stopLaunchArtifact() {
        if (!hardwareManager.getGamepad1().a) {
            hardwareManager.getLaunchServo().setPosition(LAUNCH_GATE_CLOSE);
            if(!hardwareManager.getGamepad1().left_trigger_pressed && !hardwareManager.getGamepad1().left_bumper) {
                hardwareManager.getIntakeMotor().setPower(INTAKE_NO_POWER);
            }
        }
    }

    private void clearArtifactFromLaunch() {
        if((isManualLaunchOverrideActive && hardwareManager.getGamepad2().x)
                || (!isManualLaunchOverrideActive && hardwareManager.getGamepad1().x)) {
            //hardwareManager.getLauncherMotor().setVelocity(LAUNCHER_MOTOR_IDLE_VELOCITY);
            autoLaunchArtifact();
        }
        if (hardwareManager.getGamepad1().a) {
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.BLUE.getLedValue());
            autoLaunchArtifact();
        }
    }

    private void setLaunchAngle(double setAngle) {
        if (setAngle != 0) {
            double positionValue = Range.clip(((MAX_LAUNCH_ANGLE - setAngle) * (0.65/((MAX_LAUNCH_ANGLE-MIN_LAUNCH_ANGLE)))), 0.0, 0.65);
            //telemetry.addData("servo value", positionValue);
            hardwareManager.getAngleServo().setPosition(positionValue);
        }
    }

}

