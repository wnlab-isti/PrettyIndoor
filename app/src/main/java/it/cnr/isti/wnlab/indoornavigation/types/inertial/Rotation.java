package it.cnr.isti.wnlab.indoornavigation.types.inertial;

import it.cnr.isti.wnlab.indoornavigation.types.DecomposedSensorData;

/**
 * rad/s
 */

public class Rotation extends DecomposedSensorData {

    public Rotation(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }


    @Override
    public String toString() {
        return "R " + timestamp + " " + x + " " + y + " " + z;
    }
}
