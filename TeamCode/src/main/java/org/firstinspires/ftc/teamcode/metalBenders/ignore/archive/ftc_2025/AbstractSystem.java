package org.firstinspires.ftc.teamcode.metalBenders.ignore.archive.ftc_2025;

import java.util.Locale;
import java.util.Map;

public abstract class AbstractSystem implements Runnable {

    public final HardwareManagementSystem hardwareManagementSystem;
    private final Map<String, String> telemetry;

    public AbstractSystem(HardwareManagementSystem hardwareManagementSystem, Map<String, String> telemetry) {
        this.hardwareManagementSystem = hardwareManagementSystem;
        this.telemetry = telemetry;
    }

    protected abstract void process();

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            process();
        }
    }

    public void addTelemetry(String key, String format, Object...args) {
        telemetry.put(key, String.format(Locale.ENGLISH, format, args));
    }

    public void addTelemetry(String key, String value) {
        telemetry.put(key, value);
    }
}
