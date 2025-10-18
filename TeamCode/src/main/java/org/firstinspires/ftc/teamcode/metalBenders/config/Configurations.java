package org.firstinspires.ftc.teamcode.metalBenders.config;

import java.util.HashMap;
import java.util.Map;

public final class Configurations {

    private static final Configurations INSTANCE = new Configurations();

    private Configurations() {
        //load configurations
        //apply overrides

    }

    private Configurations getInstance() {
        return INSTANCE;
    }

    private final Map<String, String> configurations = new HashMap();

    private String getOrDefaultStringConfiguration(String configName, String defaultValue) {
        return configurations.getOrDefault(configName, defaultValue);
    }

    private String getStringConfiguration(String configName) {
        return configurations.get(configName);
    }

    private int getIntegerConfiguration(String configName) {
        return Integer.parseInt(getStringConfiguration(configName));
    }

    private int getOrDefaultIntegerConfiguration(String configName, int defaultValue) {
        return configurations.containsKey(configName)
                ? getIntegerConfiguration(configName)
                : defaultValue;
    }

    private double getDoubleConfiguration(String configName) {
        return Double.parseDouble(getStringConfiguration(configName));
    }

    private double getOrDefaultDoubleConfiguration(String configName, double defaultValue) {
        return configurations.containsKey(configName)
                ? getDoubleConfiguration(configName)
                : defaultValue;
    }
}
