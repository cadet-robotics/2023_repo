package frc.robot.utils;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceSensor {
    private final AnalogInput sensor;

    public DistanceSensor(int channel) {
        sensor = new AnalogInput(channel);
    }

    public double getDistanceCm() {
        return 27.726 * Math.pow(sensor.getAverageVoltage(), -1.2045);
    }

    // both units are in centimeters
    public boolean isWithinRange(double lower, double higher) {
        double distance = getDistanceCm();
        return lower < distance && higher > distance;
    }
}
