package org.firstinspires.ftc.teamcode.metalBenders.season.decode2025_2026;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.ARM_LIMIT_SENSOR;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.AS5600_SENSOR;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.EXTEND_ARM_MOTOR;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.GRIPPER_SERVO;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.LEFT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.LEFT_REAR_MOTOR;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.LIFT_MOTOR1;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.LIFT_MOTOR2;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.RAISE_ARM_MOTOR;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.RIGHT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.RIGHT_REAR_MOTOR;
import static org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants.WRIST_SERVO;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.metalBenders.archive.core.AS5600;
import org.firstinspires.ftc.teamcode.metalBenders.archive.ftc_2025.DriveModeConstants;

import java.util.List;

public class HardwareManagementSystem {
    private final IMU imu;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx rightRearMotor;
    private final DcMotorEx leftRearMotor;
    private final Gamepad gamepad;

    public HardwareManagementSystem(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.imu = hardwareMap.get(IMU.class, DriveModeConstants.IMU);
        this.rightFrontMotor = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_MOTOR);
        this.leftFrontMotor = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_MOTOR);
        this.rightRearMotor = hardwareMap.get(DcMotorEx.class, RIGHT_REAR_MOTOR);
        this.leftRearMotor = hardwareMap.get(DcMotorEx.class, LEFT_REAR_MOTOR);
        initializeHardware();
        initializeIMU();
    }

    private void initializeHardware() {
        for(DcMotor motor : List.of(rightFrontMotor, rightRearMotor, leftFrontMotor, leftRearMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
        }
    }

    private void initializeIMU() {
        ImuOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(DOWN, BACKWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        try {
            sleep(1000);
        } catch (InterruptedException ex) {
            //do nothing
        }
    }

    public IMU getImu() {
        return imu;
    }

    public DcMotorEx getRightFrontMotor() {
        return rightFrontMotor;
    }

    public DcMotorEx getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public DcMotorEx getRightRearMotor() {
        return rightRearMotor;
    }

    public DcMotorEx getLeftRearMotor() {
        return leftRearMotor;
    }

    public Gamepad getGamepad() {
        return gamepad;
    }
}
