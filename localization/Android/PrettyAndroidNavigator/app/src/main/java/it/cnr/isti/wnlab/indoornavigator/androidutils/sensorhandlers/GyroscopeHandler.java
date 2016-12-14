package it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigator.framework.types.AngularVelocity;

/**
 * Handler for Android's gyroscope events.
 */
public class GyroscopeHandler extends SensorDataEmitter<AngularVelocity> {

    public GyroscopeHandler(SensorManager manager, int delay) {
        super(manager, Sensor.TYPE_GYROSCOPE, delay);
    }

    @Override
    protected AngularVelocity adapt(SensorEvent sensorEvent) {
        return new AngularVelocity(
                sensorEvent.values[0], // x
                sensorEvent.values[1], // y
                sensorEvent.values[2], // z
                sensorEvent.accuracy,
                sensorEvent.timestamp
        );
    }

}
