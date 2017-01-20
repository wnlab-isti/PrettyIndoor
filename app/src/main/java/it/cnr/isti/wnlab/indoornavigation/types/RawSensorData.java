package it.cnr.isti.wnlab.indoornavigation.types;

/**
 * Incapsulates common informations between sensor data types.
 */

public abstract class RawSensorData implements RawData {

    public final long timestamp;
    public final float accuracy;

    protected RawSensorData(float accuracy, long timestamp) {
        this.accuracy = accuracy;
        this.timestamp = timestamp;
    }
}
