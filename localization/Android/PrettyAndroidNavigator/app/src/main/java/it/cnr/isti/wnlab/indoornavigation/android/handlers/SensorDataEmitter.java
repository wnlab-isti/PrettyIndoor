package it.cnr.isti.wnlab.indoornavigation.android.handlers;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawSensorData;

/**
 * Abstract class that manages SensorManager, event listening and subscribers.
 */
public abstract class SensorDataEmitter<T extends RawSensorData>
        extends DataEmitter<T>
        implements SensorEventListener
{
    private Sensor mSensor;
    private SensorManager mManager;
    private int mDelay;
    private boolean active = false;

    /**
     * @param manager Android's SensorManager
     * @param sensorType Android's sensor type.
     * @param delay Delay of sensor listening.
     * @throws InvalidSensorException If sensor cannot be retrieved.
     */
    protected SensorDataEmitter(SensorManager manager,
                                int sensorType, int delay)
            throws InvalidSensorException {
        // Retrieve sensor from manager
        if ((mSensor = manager.getDefaultSensor(sensorType)) == null)
            throw new InvalidSensorException(InvalidSensorException.NULL_SENSOR);
        mManager = manager;
        mDelay = delay;
    }

    protected abstract T adapt(SensorEvent event);

    // DataEmitter overrides

    @Override
    public void startEmission() {
        if(!active) {
            mManager.registerListener(this, mSensor, mDelay);
            active = true;
        }
    }

    @Override
    public void stopEmission() {
        if(active) {
            mManager.unregisterListener(this, mSensor);
            active = false;
        }
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
