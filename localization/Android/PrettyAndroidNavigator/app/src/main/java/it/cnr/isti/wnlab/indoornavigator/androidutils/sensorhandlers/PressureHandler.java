package it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.types.Pressure;

/**
 * Handler for Android's pressure events.
 */
public class PressureHandler extends AbstractAOSPSensorEmitter<Pressure> {

    public PressureHandler(SensorManager manager, DataObserver<Pressure> receiver, int delay) {
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
