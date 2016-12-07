package it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import java.util.ArrayList;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigator.androidutils.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigator.framework.DataEmitter;
import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.types.RawSensorData;

/**
 * Abstract class that manages SensorManager, event listening and subscribers.
 */
public abstract class AbstractAOSPSensorEmitter<T extends RawSensorData>
        extends AbstractEmitter<T>
        implements SensorEventListener
{
    private Sensor mSensor;
    private SensorManager mManager;
    private int mDelay;

    /**
     * @param manager Android's SensorManager
     * @param sensorType Android's sensor type.
     * @param delay Delay of sensor listening.
     * @throws SensorException If sensor cannot be retrieved.
     */
    protected AbstractAOSPSensorEmitter(SensorManager manager,
                                        int sensorType, int delay)
            throws SensorException {
        // Retrieve sensor from manager
        if ((mSensor = manager.getDefaultSensor(sensorType)) == null)
            throw new SensorException(SensorException.NULL_SENSOR);
        mManager = manager;
        mDelay = delay;
    }

    protected abstract T adapt(SensorEvent event);

    // DataEmitter overrides

    @Override
    public void start() {
        mManager.registerListener(this, mSensor, mDelay);
    }

    @Override
    public void stop() {
        mManager.unregisterListener(this, mSensor);
    }

    // SensorEventListener overrides

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        notifyToAll(adapt(sensorEvent));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
        // do nothing for now
    }
}
