package it.cnr.isti.wnlab.indoornavigation.android.sensors;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.AngularSpeed;

/**
 * Handler for Android's gyroscope events.
 */
public class GyroscopeHandler extends SensorDataEmitter<AngularSpeed> {

    public GyroscopeHandler(SensorManager manager, int delay) {
        super(manager, Sensor.TYPE_GYROSCOPE, delay);
    }

    @Override
    protected AngularSpeed adapt(SensorEvent sensorEvent) {
        return new AngularSpeed(
                sensorEvent.values[0], // x
                sensorEvent.values[1], // y
                sensorEvent.values[2], // z
                sensorEvent.accuracy,
                sensorEvent.timestamp
        );
    }

}
