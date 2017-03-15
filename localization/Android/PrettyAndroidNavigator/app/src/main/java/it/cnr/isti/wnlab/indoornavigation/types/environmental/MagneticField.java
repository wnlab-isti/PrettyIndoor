package it.cnr.isti.wnlab.indoornavigation.types.environmental;

import it.cnr.isti.wnlab.indoornavigation.types.DecomposedSensorData;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

/**
 * μT
 */

public class MagneticField extends DecomposedSensorData {
    public MagneticField(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }

    @Override
    public String toString() {
        return "M" + RawData.LOG_SEPARATOR + timestamp + RawData.LOG_SEPARATOR + x + RawData.LOG_SEPARATOR + y + RawData.LOG_SEPARATOR + z;
    }
}
