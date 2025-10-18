package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.metalBenders.config.ColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.AbstractSystem;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.HardwareManager;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public abstract class AbstractBaseOpMode extends LinearOpMode {

    private HardwareManager hardwareManager;
    abstract protected ColorEnum getTargetColor();
    abstract protected List<AbstractSystem> getSystems();

    private List<Thread> systemThreads;
    private boolean isSystemThreadsStarted = false;
    private final Map<String, String> telemetryData = new ConcurrentHashMap<>();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
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
        hardwareManager = new HardwareManager(hardwareMap, gamepad1);
        for(AbstractSystem abstractSystem : getSystems()) {
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

    public void addTelemetry(String key, String format, Object...args) {
        telemetryData.put(key, String.format(Locale.ENGLISH, format, args));
    }

    public void addTelemetry(String key, String value) {
        telemetryData.put(key, value);
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

    public HardwareManager getHardwareManager() {
        return hardwareManager;
    }
}
