package it.cnr.isti.wnlab.indoornavigator.types;

/**
 * μT
 */

public class MagneticField extends XYZData {
    public MagneticField(float x, float y, float z, float accuracy, long timestamp) {
        super(x, y, z, accuracy, timestamp);
    }

    @Override
    public String toString() {
        return "M " + timestamp + " " + x + " " + y + " " + z;
    }
}
