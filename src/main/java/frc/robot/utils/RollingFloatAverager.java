package frc.robot.utils;

import java.util.ArrayList;

public class RollingFloatAverager {
    private ArrayList<Float> samples;

    private int n;

    public RollingFloatAverager(int n) {
        this.n = n;
        samples = new ArrayList<>();
    }

    public Float getAverage() {
        Float ret = 0f;
        
        for (int i = 0; i < samples.size(); i++) {
            ret += samples.get(i);
        }

        return ret;
    }

    public Float addSample(Float sample) {
        samples.add(sample);
        if (samples.size() > n) {
            samples.remove(0);
        }

        return getAverage();
    }

    public Float getLastSample() {
        return samples.get(samples.size() - 1);
    }

    public Float getAverageRate() {
        if (samples.size() <= 0) {
            return 0f;
        }

        Float rate = 0f;
        for (int i = 1; i < samples.size(); i++) {
            rate += samples.get(i) - samples.get(i - 1);
        }

        return rate / (float)(samples.size() - 1);
    }

    public ArrayList<Float> getSamples() {
        return samples;
    }
}
