package it.cnr.isti.wnlab.indoornavigator.android.sensorhandlers;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigator.observers.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.types.Pressure;

/**
 * Handler for Android's pressure events.
 */
public class PressureHandler extends SensorDataEmitter<Pressure> {

    public PressureHandler(SensorManager manager, DataObserver<Pressure> observer, int delay) {
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
