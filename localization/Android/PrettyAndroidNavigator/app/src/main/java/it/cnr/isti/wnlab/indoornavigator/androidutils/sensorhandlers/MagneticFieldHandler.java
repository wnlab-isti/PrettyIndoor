package it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigator.framework.types.MagneticField;

/**
 * Handler for Android's magnetometer events.
 */
public class MagneticFieldHandler extends AbstractAOSPSensorEmitter<MagneticField> {

    public MagneticFieldHandler(SensorManager manager, int delay) {
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
