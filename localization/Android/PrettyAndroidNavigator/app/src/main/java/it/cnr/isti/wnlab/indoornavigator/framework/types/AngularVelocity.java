package it.cnr.isti.wnlab.indoornavigator.framework.types;

/**
 * rad/s
 */

public class AngularVelocity extends XYZData {

    public AngularVelocity(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }

    @Override
    public String toString() {
        return "RS " + timestamp + " " + x + " " + y + " " + z;
    }
}
