package it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental;

import java.io.Serializable;

import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawData;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawSensorData;

/**
 * hPa or mBar
 */

public class Pressure extends RawSensorData implements Serializable {

    public final float pressure;

    public Pressure(float pressure, float accuracy, long timestamp) {
        super(accuracy, timestamp);
        this.pressure = pressure;
    }

    @Override
    public String toString() {
        return "P" + RawData.LOG_SEPARATOR + timestamp + RawData.LOG_SEPARATOR + pressure;
    }

}
