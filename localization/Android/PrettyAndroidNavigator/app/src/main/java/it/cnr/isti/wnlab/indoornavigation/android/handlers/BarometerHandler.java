package it.cnr.isti.wnlab.indoornavigation.android.handlers;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.Pressure;

/**
 * Handler for Android's pressure events.
 */
public class BarometerHandler extends SensorDataEmitter<Pressure> {

    public BarometerHandler(SensorManager manager, DataObserver<Pressure> observer, int delay) {
        super(manager, Sensor.TYPE_PRESSURE, delay);
    }

    @Override
    protected Pressure adapt(SensorEvent sensorEvent) {
        return new Pressure(
                sensorEvent.values[0], // pressure
                sensorEvent.accuracy,
                sensorEvent.timestamp
        );
    }

}
