package it.cnr.isti.wnlab.indoornavigation.types.inertial;

import it.cnr.isti.wnlab.indoornavigation.types.DecomposedSensorData;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

/**
 * m/s^2
 */

public class Acceleration extends DecomposedSensorData {
    public Acceleration(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }

    @Override
    public String toString() {
        return "A" + RawData.LOG_SEPARATOR + timestamp + RawData.LOG_SEPARATOR + x + RawData.LOG_SEPARATOR + y + RawData.LOG_SEPARATOR + z;
    }
}
