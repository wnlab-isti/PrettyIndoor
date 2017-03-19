package it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental;

import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawData;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawSensorData;

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
        return "H" + RawData.LOG_SEPARATOR + timestamp + RawData.LOG_SEPARATOR + humidity;
    }

}
