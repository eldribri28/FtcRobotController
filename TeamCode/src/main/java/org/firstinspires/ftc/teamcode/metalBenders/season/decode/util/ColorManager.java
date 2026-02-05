package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.GREEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.UNKNOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MINIMUM_COLOR_HIT_COUNT_TO_CHANGE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.IndicatorLedEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class ColorManager implements Runnable {
    private final HardwareManager hardwareManager;
    private final ColorHitCount intakeColorHitCount;
    private final ColorHitCount launcherColorHitCount;
    private ArtifactColorEnum intakeArtifactColor = UNKNOWN;
    private ArtifactColorEnum launcherArtifactColor = UNKNOWN;
    private final ReentrantReadWriteLock intakeArtifactColorLock = new ReentrantReadWriteLock();
    private final ReentrantReadWriteLock launcherArtifactColorLock = new ReentrantReadWriteLock();
    private final Map<String, String> telemetry = new ConcurrentHashMap<>();

    public ColorManager(HardwareManager hardwareManager) {
        this.hardwareManager = hardwareManager;
        this.intakeColorHitCount = new ColorHitCount();
        this.launcherColorHitCount = new ColorHitCount();
    }

    public void setArtifactColors() {
        setLauncherArtifactColor();
        setIntakeArtifactColor();
    }

    private void setLauncherArtifactColor() {
        incrementColorHitCounts(launcherColorHitCount, hardwareManager.getLaunchColorSensor(), hardwareManager.getLaunchColorSensor2());
        if(launcherArtifactColor == UNKNOWN || launcherColorHitCount.isMinimumHitCountReached()) {
            setLauncherArtifactColor(launcherColorHitCount.getArtifactColorEnum());
            if(launcherArtifactColor == GREEN) {
                hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.GREEN.getLedValue());
            } else if (launcherArtifactColor == PURPLE){
                hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.PURPLE.getLedValue());
            } else {
                hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.BLACK.getLedValue());
            }
        }
        telemetry.put("Launcher Color", getLauncherArtifactColor().name());
    }

    private void setIntakeArtifactColor() {
        incrementColorHitCounts(intakeColorHitCount, hardwareManager.getIntakeColorSensor());
        if(intakeArtifactColor == UNKNOWN || intakeColorHitCount.isMinimumHitCountReached()) {
            setIntakeArtifactColor(intakeColorHitCount.getArtifactColorEnum());
        }
        telemetry.put("Intake Color", getIntakeArtifactColor().name());
    }

    private void incrementColorHitCounts(ColorHitCount colorHitCount, RevColorSensorV3 ... colorSensors) {
        List<ArtifactColorEnum> colors = new ArrayList<>();
        for(RevColorSensorV3 colorSensor : colorSensors) {
            colors.add(getArtifactColor(colorSensor));
        }
        ArtifactColorEnum color;
        if(colors.stream().anyMatch(artifactColorEnum -> artifactColorEnum == GREEN)) {
            color = GREEN;
        } else if (colors.stream().anyMatch(artifactColorEnum -> artifactColorEnum == ArtifactColorEnum.PURPLE)) {
            color = ArtifactColorEnum.PURPLE;
        } else {
            color = ArtifactColorEnum.NONE;
        }
        colorHitCount.increment(color);
    }

    private ArtifactColorEnum getArtifactColor(RevColorSensorV3 colorSensor) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;

        ArtifactColorEnum artifactColorEnum;
        if (blue > green && red > 0.2 && blue > 0.2) {
            artifactColorEnum = ArtifactColorEnum.PURPLE;
        } else if (green > red && green > blue && green > 0.2) {
            artifactColorEnum = GREEN;
        } else {
            artifactColorEnum = ArtifactColorEnum.NONE;
        }
        return artifactColorEnum;
    }

    public ArtifactColorEnum getLauncherArtifactColor() {
        launcherArtifactColorLock.readLock().lock();
        try {
            return launcherArtifactColor;
        } finally {
            launcherArtifactColorLock.readLock().unlock();
        }
    }

    private void setLauncherArtifactColor(ArtifactColorEnum artifactColor) {
        launcherArtifactColorLock.writeLock().lock();
        try {
            this.launcherArtifactColor = artifactColor;
        } finally {
            launcherArtifactColorLock.writeLock().unlock();
        }
    }

    public ArtifactColorEnum getIntakeArtifactColor() {
        intakeArtifactColorLock.readLock().lock();
        try {
            return intakeArtifactColor;
        } finally {
            intakeArtifactColorLock.readLock().unlock();
        }
    }

    private void setIntakeArtifactColor(ArtifactColorEnum artifactColor) {
        intakeArtifactColorLock.writeLock().lock();
        try {
            this.intakeArtifactColor = artifactColor;
        } finally {
            intakeArtifactColorLock.writeLock().unlock();
        }
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            setArtifactColors();
        }
    }

    private static class ColorHitCount {
        private ArtifactColorEnum artifactColorEnum;
        private int hitCount = 0;

        public void increment(ArtifactColorEnum artifactColorEnum) {
            if(this.artifactColorEnum == artifactColorEnum) {
                hitCount++;
            } else {
                this.artifactColorEnum = artifactColorEnum;
                hitCount = 1;
            }
        }

        public ArtifactColorEnum getArtifactColorEnum() {
            return artifactColorEnum;
        }

        public boolean isMinimumHitCountReached() {
            return hitCount >= MINIMUM_COLOR_HIT_COUNT_TO_CHANGE;
        }
    }

    public Map<String, String> getTelemetry() {
        return new HashMap<>(telemetry);
    }
}
