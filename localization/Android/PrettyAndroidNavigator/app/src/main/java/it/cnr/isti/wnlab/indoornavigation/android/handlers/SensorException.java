package it.cnr.isti.wnlab.indoornavigation.android.handlers;

public class SensorException extends RuntimeException {

    public static final String NULL_SENSOR = "Can't retrieve specified sensor.";

    public SensorException(String s) {
        super(s);
    }

}
