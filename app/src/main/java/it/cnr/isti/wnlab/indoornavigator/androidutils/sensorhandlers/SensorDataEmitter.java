package it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigator.framework.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigator.framework.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigator.framework.types.RawSensorData;

/**
 * Abstract class that manages SensorManager, event listening and subscribers.
 */
public abstract class SensorDataEmitter<T extends RawSensorData>
        extends AbstractEmitter<T>
        implements StartableStoppable, SensorEventListener
{
    private Sensor mSensor;
    private SensorManager mManager;
    private int mDelay;

    protected boolean started = false;

    /**
     * @param manager Android's SensorManager
     * @param sensorType Android's sensor type.
     * @param delay Delay of sensor listening.
     * @throws SensorException If sensor cannot be retrieved.
     */
    protected SensorDataEmitter(SensorManager manager,
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
        if(!started) {
            mManager.registerListener(this, mSensor, mDelay);
            started = true;
        }
    }

    @Override
    public void stop() {
        if(started) {
            mManager.unregisterListener(this, mSensor);
            started = false;
        }
    }

    @Override
    public boolean isStarted() {
        return started;
    }

    // SensorEventListener overrides

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        notifyObservers(adapt(sensorEvent));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
        // do nothing for now
    }
}
