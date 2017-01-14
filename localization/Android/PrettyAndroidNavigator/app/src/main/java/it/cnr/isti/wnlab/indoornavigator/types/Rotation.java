package it.cnr.isti.wnlab.indoornavigator.types;

/**
 * rad/s
 */

public class Rotation extends XYZData {

    public Rotation(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }


    @Override
    public String toString() {
        return "R " + timestamp + " " + x + " " + y + " " + z;
    }
}
