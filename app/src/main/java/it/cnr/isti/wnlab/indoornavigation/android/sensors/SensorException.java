package it.cnr.isti.wnlab.indoornavigation.android.sensors;

public class SensorException extends RuntimeException {

    public static final String NULL_SENSOR = "Can't retrieve specified sensor.";

    public SensorException(String s) {
        super(s);
    }

}
