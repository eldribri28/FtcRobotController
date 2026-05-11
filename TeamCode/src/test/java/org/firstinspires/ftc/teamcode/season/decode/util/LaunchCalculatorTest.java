package org.firstinspires.ftc.teamcode.season.decode.util;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.junit.Test;

public class LaunchCalculatorTest {

    @Test
    public void testGetLaunchData() {
        LaunchCalculator.LaunchResult launchResult = LaunchCalculator.getLaunchData(0.3, 1.175, 0.5, 2000, 0);
        System.out.println();
        System.out.println(launchResult);
        System.out.println();
    }
}
