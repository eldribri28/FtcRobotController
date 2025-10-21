package org.firstinspires.ftc.teamcode.metalBenders.ignore.systems;

import org.firstinspires.ftc.teamcode.metalBenders.ignore.data.DataManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.types.SystemPriorityEnum;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.types.SystemStateEnum;

import java.util.Locale;
import java.util.Map;

public abstract class AbstractSystem implements Runnable {
    private SystemStateEnum state = SystemStateEnum.NEW;
    private HardwareManager hardwareManager;
    private DataManager dataManager;
    private Map<String, String> telemetryData;

    public abstract SystemPriorityEnum getSystemPriority();
    protected abstract void start();
    protected abstract void process();
    protected abstract void shutdown();

    @Override
    public void run() {
        setState(SystemStateEnum.STARTING);
        start();
        setState(SystemStateEnum.RUNNING);
        while (!Thread.currentThread().isInterrupted()) {
            try {
                process();
            }catch (Exception e) {
                addTelemetry(
                        this.getClass().getSimpleName() + " exception",
                        e.getClass().getSimpleName() + " - " + e.getMessage());
            }
        }
        setState(SystemStateEnum.SHUTTING_DOWN);
        shutdown();
    }

    public HardwareManager getHardwareManager() {
        return hardwareManager;
    }

    public void setHardwareManager(HardwareManager hardwareManager) {
        this.hardwareManager = hardwareManager;
    }

    public DataManager getDataManager() {
        return dataManager;
    }

    public void setDataManager(DataManager dataManager) {
        this.dataManager = dataManager;
    }

    public void setTelemetryData(Map<String, String> telemetryData) {
        this.telemetryData = telemetryData;
    }

    public void addTelemetry(String key, String format, Object...args) {
        telemetryData.put(key, String.format(Locale.ENGLISH, format, args));
    }

    public void addTelemetry(String key, String value) {
        telemetryData.put(key, value);
    }

    public void addTelemetry(String key, double value) {
        addTelemetry(key, "%.2f", value);
    }

    public void addTelemetry(String key, int value) {
        addTelemetry(key, String.valueOf(value));
    }

    public void setState(SystemStateEnum state) {
        this.state = state;
        addTelemetry("State " + this.getClass().getSimpleName(), state.name());
    }

    public SystemStateEnum getState() {
        return state;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
