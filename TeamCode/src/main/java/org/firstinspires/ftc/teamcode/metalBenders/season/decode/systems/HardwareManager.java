package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareEnum;

import java.util.List;

public class HardwareManager {
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx rightRearMotor;
    private final DcMotorEx leftRearMotor;
    private final Gamepad gamepad;

    public HardwareManager(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.rightFrontMotor = hardwareMap.get(DcMotorEx.class, HardwareEnum.RIGHT_FRONT_MOTOR.getName());
        this.leftFrontMotor = hardwareMap.get(DcMotorEx.class, HardwareEnum.LEFT_FRONT_MOTOR.getName());
        this.rightRearMotor = hardwareMap.get(DcMotorEx.class, HardwareEnum.RIGHT_REAR_MOTOR.getName());
        this.leftRearMotor = hardwareMap.get(DcMotorEx.class, HardwareEnum.LEFT_REAR_MOTOR.getName());
        initializeHardware();
    }

    private void initializeHardware() {
        for(DcMotor motor : List.of(rightFrontMotor, rightRearMotor, leftFrontMotor, leftRearMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
        }
    }

    public Gamepad getGamepad() {
        return gamepad;
    }
}
