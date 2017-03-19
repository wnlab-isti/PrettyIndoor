package it.cnr.isti.wnlab.indoornavigation.android.sensors;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.Acceleration;

/**
 * Handler for Android's accelerometer events.
 */
public class AccelerometerHandler extends SensorDataEmitter<Acceleration> {

    public AccelerometerHandler(SensorManager manager, int delay) {
        super(manager, Sensor.TYPE_ACCELEROMETER, delay);
    }

    @Override
    protected Acceleration adapt(SensorEvent sensorEvent) {
        return new Acceleration(
                sensorEvent.values[0], // x
                sensorEvent.values[1], // y
                sensorEvent.values[2], // z
                sensorEvent.accuracy,
                sensorEvent.timestamp
        );
    }

}
