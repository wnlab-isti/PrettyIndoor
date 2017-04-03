package it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial;

import java.io.Serializable;

import it.cnr.isti.wnlab.indoornavigation.javaonly.types.DecomposedSensorData;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawData;

/**
 * rad/s
 */

public class AngularSpeed extends DecomposedSensorData implements Serializable {

    public AngularSpeed(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }

    @Override
    public String toString() {
        return "R" + RawData.LOG_SEPARATOR + timestamp + RawData.LOG_SEPARATOR + x + RawData.LOG_SEPARATOR + y + RawData.LOG_SEPARATOR + z;
    }
}
