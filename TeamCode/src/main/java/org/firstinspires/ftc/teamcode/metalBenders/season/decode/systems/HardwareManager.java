package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.UP;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareEnum;

import java.util.List;

public class HardwareManager {
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx rightRearMotor;
    private final DcMotorEx leftRearMotor;
    private final DcMotorEx intakeMotor;
    private final Gamepad gamepad;
    private final IMU imu;

    public HardwareManager(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.rightFrontMotor = hardwareMap.get(DcMotorEx.class, HardwareEnum.RIGHT_FRONT_MOTOR.getName());
        this.leftFrontMotor = hardwareMap.get(DcMotorEx.class, HardwareEnum.LEFT_FRONT_MOTOR.getName());
        this.rightRearMotor = hardwareMap.get(DcMotorEx.class, HardwareEnum.RIGHT_REAR_MOTOR.getName());
        this.leftRearMotor = hardwareMap.get(DcMotorEx.class, HardwareEnum.LEFT_REAR_MOTOR.getName());
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, HardwareEnum.INTAKE_MOTOR.getName());
        this.imu = hardwareMap.get(IMU.class, HardwareEnum.IMU.getName());
        initializeHardware();
    }

    private void initializeHardware() {
        for(DcMotor motor : List.of(leftFrontMotor, leftRearMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        for(DcMotor motor : List.of(rightFrontMotor, rightRearMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        initializeIMU();
    }

    private void initializeIMU() {
        ImuOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RIGHT, UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        try {
            sleep(1000);
        } catch(Exception e){
        }
    }

    public Gamepad getGamepad() {
        return gamepad;
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

    public IMU getImu() {
        return imu;
    }

    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }
}
