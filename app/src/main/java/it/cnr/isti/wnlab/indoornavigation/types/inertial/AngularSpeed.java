package it.cnr.isti.wnlab.indoornavigation.types.inertial;

import it.cnr.isti.wnlab.indoornavigation.types.DecomposedSensorData;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

/**
 * rad/s
 */

public class AngularSpeed extends DecomposedSensorData {

    public AngularSpeed(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }

    @Override
    public String toString() {
        return "R" + RawData.LOG_SEPARATOR + timestamp + RawData.LOG_SEPARATOR + x + RawData.LOG_SEPARATOR + y + RawData.LOG_SEPARATOR + z;
    }
}
