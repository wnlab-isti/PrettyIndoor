package it.cnr.isti.wnlab.indoornavigation.android.handlers;

public class InvalidSensorException extends IllegalArgumentException {

    public static final String NULL_SENSOR = "Can't retrieve specified sensor.";

    public InvalidSensorException(String s) {
        super(s);
    }

}
