package org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

public final class PropertyManager {

    private static final String PROPERTY_OVERRIDE_FILENAME = "overrides.txt";
    private static final String PROPERTIES_FILENAME = "properties.txt";
    private static final PropertyManager INSTANCE = new PropertyManager();
    private final Properties properties;

    private PropertyManager() {

        Properties defaultProperties = new Properties();
        try (FileInputStream fis = new FileInputStream(PROPERTIES_FILENAME)) {
            defaultProperties.load(fis);
        } catch (IOException e) {
            System.err.println("Error loading configuration file: " + e.getMessage());
        }

        properties = new Properties(defaultProperties);
        try (FileInputStream fis = new FileInputStream(PROPERTY_OVERRIDE_FILENAME)) {
            properties.load(fis);
        } catch (IOException e) {
            System.err.println("Error loading configuration file: " + e.getMessage());
        }
    }

    private PropertyManager getInstance() {
        return INSTANCE;
    }

    private int getIntProperty(String key, int defaultValue) {
        String propertyValue = properties.getProperty(key);
        if(propertyValue != null) {
            try {
                return Integer.parseInt(propertyValue);
            } catch (Exception e){}
        }
        return defaultValue;
    }

    private double getDoubleProperty(String key, double defaultValue) {
        String propertyValue = properties.getProperty(key);
        if(propertyValue != null) {
            try {
                return Double.parseDouble(propertyValue);
            } catch (Exception e){}
        }
        return defaultValue;
    }
}
