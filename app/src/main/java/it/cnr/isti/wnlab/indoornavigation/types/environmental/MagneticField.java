package it.cnr.isti.wnlab.indoornavigation.types.environmental;

import it.cnr.isti.wnlab.indoornavigation.types.DecomposedSensorData;

/**
 * μT
 */

public class MagneticField extends DecomposedSensorData {
    public MagneticField(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }

    @Override
    public String toString() {
        return "M " + timestamp + " " + x + " " + y + " " + z;
    }
}