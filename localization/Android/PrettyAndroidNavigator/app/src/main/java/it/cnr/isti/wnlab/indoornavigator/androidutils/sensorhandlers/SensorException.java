package it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers;

public class SensorException extends RuntimeException {

    public static final String NULL_SENSOR = "Can't retrieve specified sensor.";

    public SensorException(String s) {
        super(s);
    }

}
