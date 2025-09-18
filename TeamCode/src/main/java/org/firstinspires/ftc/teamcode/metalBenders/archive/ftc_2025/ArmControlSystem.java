package org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;

public class ArmControlSystem extends AbstractSystem {

    private enum RaiseArmState {
        RAISING,
        LOWERING,
        NO_ACTION
    }

    private enum LiftArmState {
        LIFTING,
        LOWERING,
        NO_ACTION
    }

    private enum ExtendArmState {
        EXTENDING,
        RETRACTING,
        MAINTAIN_POSITION
    }

    private enum IntakeState {
        IN,
        OUT,
        OFF
    }

    private double currentArmAngle;
    private WristPosition wristPosition;
    private IntakeState intakeState;
    private int maxArmExtensionPosition;
//    private double armLength;
    private RaiseArmState raiseArmState;
    private LiftArmState liftArmState;
    private ExtendArmState extendArmState;
    private Integer extendArmTargetPosition;

    public ArmControlSystem(HardwareManagementSystem hardwareManagementSystem, Map<String, String> telemetry) {
        super(hardwareManagementSystem, telemetry);
    }

    @Override
    protected void process() {
//        calculateArmLengthInches(hardwareManagementSystem.getAs5600sensor().getAngleDeg());
        currentArmAngle = hardwareManagementSystem.getAs5600sensor().getAngleDeg();
        calculateMaxArmExtensionPosition();
        setRaiseArmPower();
        setLiftArmPower();
        setExtendArmPower();
        setWristPosition();
        setIntakePowerAndDirection();
    }

    private void setRaiseArmPower() {
        double power = hardwareManagementSystem.getGamepad().right_stick_y;
        if(hardwareManagementSystem.getGamepad().right_stick_y > 0) {
            raiseArmState = RaiseArmState.LOWERING;
        } else if (hardwareManagementSystem.getGamepad().right_stick_y < 0) {
            raiseArmState = RaiseArmState.RAISING;
        } else {
            raiseArmState = RaiseArmState.NO_ACTION;
        }
        hardwareManagementSystem.getRaiseArmMotor().setPower(power * FIFTY_PERCENT_POWER);
        addTelemetry("Arm - Raise State", raiseArmState.name());
    }

    private void setLiftArmPower() {
        double power;
        if(hardwareManagementSystem.getGamepad().dpad_right) {
            liftArmState = LiftArmState.LIFTING;
            power = FULL_POWER;
        } else if (hardwareManagementSystem.getGamepad().dpad_left) {
            liftArmState = LiftArmState.LOWERING;
            power = FULL_POWER_REVERSE;
        } else {
            liftArmState = LiftArmState.NO_ACTION;
            power = NO_POWER;
        }
        hardwareManagementSystem.getLiftMotor1().setPower(power);
        hardwareManagementSystem.getLiftMotor2().setPower(power);
        addTelemetry("Arm - Lift State", liftArmState.name());
    }

    private void setExtendArmPower() {
        boolean armLimitSensorPressed = hardwareManagementSystem.getArmLimitSensor().isPressed();
        if(hardwareManagementSystem.getGamepad().dpad_up) {
            extendArmTargetPosition = null;
            hardwareManagementSystem.getExtendArmMotor().setTargetPosition(maxArmExtensionPosition);
            hardwareManagementSystem.getExtendArmMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardwareManagementSystem.getExtendArmMotor().setPower(SEVENTY_PERCENT_POWER);
            extendArmState = ExtendArmState.EXTENDING;
        } else if (hardwareManagementSystem.getGamepad().dpad_down && hardwareManagementSystem.getArmLimitSensor().isPressed()) {
            extendArmTargetPosition = null;
            hardwareManagementSystem.getExtendArmMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardwareManagementSystem.getExtendArmMotor().setPower(SEVENTY_PERCENT_POWER);
            extendArmState = ExtendArmState.RETRACTING;
        } else {
            if(extendArmTargetPosition == null) {
                extendArmTargetPosition = hardwareManagementSystem.getExtendArmMotor().getTargetPosition();
                hardwareManagementSystem.getExtendArmMotor().setTargetPosition(extendArmTargetPosition);
                hardwareManagementSystem.getExtendArmMotor().setPower(NO_POWER);
                hardwareManagementSystem.getExtendArmMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendArmState = ExtendArmState.MAINTAIN_POSITION;
            }
            if(!hardwareManagementSystem.getArmLimitSensor().isPressed()) {
                hardwareManagementSystem.getExtendArmMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hardwareManagementSystem.getExtendArmMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        addTelemetry("Arm - Angle (deg)", "%.2f", hardwareManagementSystem.getAs5600sensor().getAngleDeg());
        addTelemetry("Arm - Limit Sensor Pressed", Boolean.toString(armLimitSensorPressed));
        addTelemetry("Arm - Extension State", extendArmState.name());
    }

    private void setWristPosition() {
        double currentArmAngle = hardwareManagementSystem.getAs5600sensor().getAngleDeg();
        if (hardwareManagementSystem.getGamepad().a || currentArmAngle <= 70) {
            wristPosition = WristPosition.UP;
        } else if (hardwareManagementSystem.getGamepad().b || currentArmAngle >= 145) {
            wristPosition = WristPosition.DOWN;
        } else if (hardwareManagementSystem.getGamepad().x) {
            wristPosition = WristPosition.MIDDLE;
        }
        if(wristPosition != null) {
            hardwareManagementSystem.getWristServo().setPosition(wristPosition.getValue());
            addTelemetry("Arm - Wrist position", wristPosition.name());
        }
    }

    private void setIntakePowerAndDirection() {
        if (hardwareManagementSystem.getGamepad().left_bumper) {
            hardwareManagementSystem.getIntakeWheelServo().setDirection(REVERSE);
            hardwareManagementSystem.getIntakeWheelServo().setPower(FULL_POWER);
            intakeState = IntakeState.IN;
        } else if (hardwareManagementSystem.getGamepad().right_bumper) {
            hardwareManagementSystem.getIntakeWheelServo().setDirection(FORWARD);
            hardwareManagementSystem.getIntakeWheelServo().setPower(SIXTY_PERCENT_POWER);
            intakeState = IntakeState.OUT;
        } else {
            hardwareManagementSystem.getIntakeWheelServo().setPower(NO_POWER);
            intakeState = IntakeState.OFF;
        }
        addTelemetry("Arm - Intake State", intakeState.name());
    }

    public void calculateMaxArmExtensionPosition() {
        double calculatedLimit = Math.abs(HORIZONTAL_LIMIT_MAX_ARM_LENGTH / Math.cos(Math.toRadians(currentArmAngle - ARM_90_DEG_OFFSET)));
        maxArmExtensionPosition = -(int)((Math.min(calculatedLimit, PHYSICAL_ARM_LIMIT) - ARM_COLLAPSED_MINIMUM_LENGTH) * ARM_TICKS_PER_INCH);
        addTelemetry("Arm - max position", Integer.toString(maxArmExtensionPosition));
        addTelemetry("Arm - current position", Integer.toString(hardwareManagementSystem.getExtendArmMotor().getCurrentPosition()));
    }
}
