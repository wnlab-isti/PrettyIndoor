package it.cnr.isti.wnlab.indoornavigation.android.stepdetection;

import android.hardware.SensorManager;

import it.cnr.isti.wnlab.indoornavigation.javaonly.StepDetector;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.Acceleration;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Step;


public class FasterStepDetector extends StepDetector implements Observer<Acceleration> {

    private float   mLimit = 1.9f; // Sperimentally found on my slow walk. It was 10.0 before
    private float   mLastValues[] = new float[3*2];
    private float   mScale[] = new float[2];
    private float   mYOffset;

    private float   mLastDirections[] = new float[3*2];
    private float   mLastExtremes[][] = { new float[3*2], new float[3*2] };
    private float   mLastDiff[] = new float[3*2];
    private int     mLastMatch = -1;

    private Emitter<Acceleration> mAccelerometer;

    public FasterStepDetector(Emitter<Acceleration> accelerometer) {
        int h = 480;
        mYOffset = h * 0.5f;
        mScale[0] = - (h * 0.5f * (1.0f / (SensorManager.STANDARD_GRAVITY * 2)));
        mScale[1] = - (h * 0.5f * (1.0f / (SensorManager.MAGNETIC_FIELD_EARTH_MAX)));
        mAccelerometer = accelerometer;
    }

    public void setSensitivity(float sensitivity) {
        mLimit = sensitivity; // 1.97  2.96  4.44  6.66  10.00  15.00  22.50  33.75  50.62
    }

    /**
     * Notify observers on step.
     * @param timestamp
     */
    private void onStep(long timestamp) {
        notifyObservers(new Step(timestamp));
    }

    @Override
    protected void startEmission() {
        mAccelerometer.register(this);
    }

    @Override
    protected void stopEmission() {
        mAccelerometer.unregister(this);
    }

    @Override
    public void notify(Acceleration data) {
        float vSum = 0;

        vSum += mYOffset + data.x * mScale[1];
        vSum += mYOffset + data.y * mScale[1];
        vSum += mYOffset + data.z * mScale[1];

        int k = 0;
        float v = vSum / 3;

        float direction = (v > mLastValues[k] ? 1 : (v < mLastValues[k] ? -1 : 0));
        if (direction == - mLastDirections[k]) {
            // Direction changed
            int extType = (direction > 0 ? 0 : 1); // minimum or maximum?
            mLastExtremes[extType][k] = mLastValues[k];
            float diff = Math.abs(mLastExtremes[extType][k] - mLastExtremes[1 - extType][k]);

            if (diff > mLimit) {

                boolean isAlmostAsLargeAsPrevious = diff > (mLastDiff[k]*2/3);
                boolean isPreviousLargeEnough = mLastDiff[k] > (diff/3);
                boolean isNotContra = (mLastMatch != 1 - extType);

                if (isAlmostAsLargeAsPrevious && isPreviousLargeEnough && isNotContra) {
                    onStep(System.currentTimeMillis());
                    mLastMatch = extType;
                }
                else {
                    mLastMatch = -1;
                }
            }
            mLastDiff[k] = diff;
        }
        mLastDirections[k] = direction;
        mLastValues[k] = v;
    }
}
