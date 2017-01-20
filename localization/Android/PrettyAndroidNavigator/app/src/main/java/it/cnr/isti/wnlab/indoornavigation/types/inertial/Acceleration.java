package it.cnr.isti.wnlab.indoornavigation.types.inertial;

import it.cnr.isti.wnlab.indoornavigation.types.DecomposedSensorData;

/**
 * m/s^2
 */

public class Acceleration extends DecomposedSensorData {
    public Acceleration(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }

    @Override
    public String toString() {
        return "A " + timestamp + " " + x + " " + y + " " + z;
    }
}
