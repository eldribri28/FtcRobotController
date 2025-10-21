package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.metalBenders.ignore.data.DataManager;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.AbstractSystem;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public abstract class AbstractBaseOpMode extends LinearOpMode {

    private final List<Thread> systemThreads = new ArrayList<>();
    private final Map<String, String> telemetryData = new ConcurrentHashMap<>();
    private boolean isSystemThreadsStarted = false;

    abstract protected List<AbstractSystem> getSystems();
    abstract protected AprilTagEnum getTargetAprilTag();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetryData.put("Target name", getTargetAprilTag().name());
        telemetryData.put("Target id", String.valueOf(getTargetAprilTag().getId()));
        updateTelemetry();
        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {
            if(!isSystemThreadsStarted) {
                startSystem();
            }
            updateRuntime();
            updateTelemetry();
        }
        stopSystem();
    }

    private void initialize() {
        HardwareManager hardwareManager = new HardwareManager(hardwareMap, gamepad1);
        DataManager dataManager = new DataManager();
        for(AbstractSystem abstractSystem : getSystems()) {
            abstractSystem.setHardwareManager(hardwareManager);
            abstractSystem.setTelemetryData(telemetryData);
            abstractSystem.setDataManager(dataManager);
            Thread thread = new Thread(abstractSystem);
            thread.setPriority(abstractSystem.getSystemPriority().getValue());
            systemThreads.add(thread);
        }
    }

    private void startSystem() {
        systemThreads.forEach(Thread::start);
        isSystemThreadsStarted = true;
    }

    private void stopSystem() {
        systemThreads.forEach(Thread::interrupt);
        isSystemThreadsStarted = false;
    }

    private void updateTelemetry() {
        List<String> keys = new ArrayList<>(telemetryData.keySet());
        Collections.sort(keys);
        keys.forEach(key -> telemetry.addData(key, telemetryData.get(key)));
        telemetry.update();
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetryData.put(
                "Runtime",
                String.format(
                        Locale.ENGLISH,
                        "%02.0f:%02.0f",
                        totalSeconds / 60,
                        totalSeconds % 60));
    }
}
