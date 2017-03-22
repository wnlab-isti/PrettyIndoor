package it.cnr.isti.wnlab.indoornavigation.javaonly.types;

/**
 * Abstract XYZ data type.
 */
public abstract class DecomposedSensorData extends RawSensorData {

    public final float x, y, z;

    protected DecomposedSensorData(float x, float y, float z, float accuracy, long timestamp) {
        super(accuracy, timestamp);
        // Values
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * @return a NEW 3D array with coordinates.
     */
    public float[] getArray() {
        float[] array = new float[3];
        array[0] = x;
        array[1] = y;
        array[2] = z;
        return array;
    }

}
