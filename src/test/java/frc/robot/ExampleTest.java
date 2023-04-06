package frc.robot;

import org.junit.jupiter.api.Test;

import frc.robot.utils.RollingFloatAverager;

import static org.junit.jupiter.api.Assertions.*;

class ExampleTest
{
    // To learn more about how to write unit tests, see the
    // JUnit 5 User Guide at https://junit.org/junit5/docs/current/user-guide/

    @Test
    void twoPlusTwoShouldEqualFour()
    {
        assertEquals(4, 2 + 2);
    }

    @Test
    void autoLevelRateTest() {
        RollingFloatAverager samples = new RollingFloatAverager(10);
        for (int i = 0; i < 9; i++) {
            samples.addSample(15f);
        }
        samples.addSample(24f);

        assertEquals(samples.getAverageRate(), 1);
    }
}
