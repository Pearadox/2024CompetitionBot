package frc.lib.util;

import java.util.TreeMap;

public class LerpTable {
    private TreeMap<Double, Double> datapoints = new TreeMap<>();

    public void addPoint(double key, double value) {
        datapoints.put(key, value);
    }

    public void addPoints(double[] keys, double[] values) {
        for(int i = 0; i < keys.length; i++)
            datapoints.put(keys[i], values[i]);
    }

    public double interpolate(double input) {
        var y = datapoints.ceilingEntry(input);
        var x = datapoints.floorEntry(input);
        try {
            return x.getValue() + (y.getValue() - x.getValue()) * (input - x.getKey())/(y.getKey() - x.getKey());
        } catch (NullPointerException e) {
            if (x != null) {
                return x.getValue();
            } else if (y != null) {
                return y.getValue();
            } else {
                return 0;
            }
        }
    }
}