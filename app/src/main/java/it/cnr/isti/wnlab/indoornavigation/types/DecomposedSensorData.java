package it.cnr.isti.wnlab.indoornavigation.types;

/**
 * Abstract XYZ data type.
 */
public abstract class DecomposedSensorData extends RawSensorData {

    public final float x, y, z;
    public final float[] array;

    protected DecomposedSensorData(float x, float y, float z, float accuracy, long timestamp) {
        super(accuracy, timestamp);
        // Values
        this.x = x;
        this.y = y;
        this.z = z;
        // Array representation
        this.array = new float[3];
        array[0] = x;
        array[1] = y;
        array[2] = z;
    }
}
