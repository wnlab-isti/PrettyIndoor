package it.cnr.isti.wnlab.indoornavigation.android.handlers;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigation.types.environmental.MagneticField;

/**
 * Handler for Android's magnetometer events.
 */
public class MagnetometerHandler extends SensorDataEmitter<MagneticField> {

    public MagnetometerHandler(SensorManager manager, int delay) {
        super(manager, Sensor.TYPE_MAGNETIC_FIELD, delay);
    }

    @Override
    protected MagneticField adapt(SensorEvent sensorEvent) {
        return new MagneticField(
                sensorEvent.values[0], // x
                sensorEvent.values[1], // y
                sensorEvent.values[2], // z
                sensorEvent.accuracy,
                sensorEvent.timestamp
        );
    }

}
