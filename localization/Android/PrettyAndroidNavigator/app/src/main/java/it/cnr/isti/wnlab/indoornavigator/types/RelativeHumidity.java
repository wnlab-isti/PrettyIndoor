package it.cnr.isti.wnlab.indoornavigator.types;

/**
 * %
 */

public class RelativeHumidity extends RawSensorData {

    private final float humidity;

    public RelativeHumidity(float humidity, float accuracy, long timestamp) {
        super(accuracy, timestamp);
        this.humidity = humidity;
    }

    @Override
    public String toString() {
        return "% " + timestamp + " " + humidity;
    }

}
