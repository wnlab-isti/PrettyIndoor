package it.cnr.isti.wnlab.indoornavigator.types;

/**
 * hPa or mBar
 */

public class Pressure extends RawSensorData {

    public final float pressure;

    public Pressure(float pressure, float accuracy, long timestamp) {
        super(accuracy, timestamp);
        this.pressure = pressure;
    }

    @Override
    public String toString() {
        return "P " + timestamp + " " + pressure;
    }

}
