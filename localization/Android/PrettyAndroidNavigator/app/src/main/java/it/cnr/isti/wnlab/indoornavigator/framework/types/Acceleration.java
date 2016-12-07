package it.cnr.isti.wnlab.indoornavigator.framework.types;

/**
 * m/s^2
 */

public class Acceleration extends XYZData {
    public Acceleration(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }

    @Override
    public String toString() {
        return "A " + timestamp + " " + x + " " + y + " " + z;
    }
}
