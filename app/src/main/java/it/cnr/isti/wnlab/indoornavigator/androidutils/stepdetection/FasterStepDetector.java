package it.cnr.isti.wnlab.indoornavigator.androidutils.stepdetection;

import android.hardware.SensorManager;
import android.util.Log;

import it.cnr.isti.wnlab.indoornavigator.framework.DataEmitter;
import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.types.Acceleration;
import it.cnr.isti.wnlab.indoornavigator.framework.util.StepDetectedCallback;


public class FasterStepDetector implements StepDetector {

    private float   mLimit = 10;
    private float   mLastValues[] = new float[3*2];
    private float   mScale[] = new float[2];
    private float   mYOffset;

    private float   mLastDirections[] = new float[3*2];
    private float   mLastExtremes[][] = { new float[3*2], new float[3*2] };
    private float   mLastDiff[] = new float[3*2];
    private int     mLastMatch = -1;

    private boolean started = false;

    private StepDetectedCallback mCallback;

    public FasterStepDetector(StepDetectedCallback callback, DataEmitter<Acceleration> accelerometer) {
        int h = 480; // TODO: remove this constant
        mYOffset = h * 0.5f;
        mScale[0] = - (h * 0.5f * (1.0f / (SensorManager.STANDARD_GRAVITY * 2)));
        mScale[1] = - (h * 0.5f * (1.0f / (SensorManager.MAGNETIC_FIELD_EARTH_MAX)));

        mCallback = callback;
        accelerometer.register(
                new DataObserver<Acceleration>() {
                    @Override
                    public void notify(Acceleration data) {
                        onAccelerometer(data);
                    }
                }
        );
    }

    public void setSensitivity(float sensitivity) {
        mLimit = sensitivity; // 1.97  2.96  4.44  6.66  10.00  15.00  22.50  33.75  50.62
    }

    private void onAccelerometer(Acceleration data) {
        float vSum = 0;
        float[] values = new float[3];
        values[0] = data.x;
        values[1] = data.y;
        values[2] = data.z;

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
                    if(started) {
                        mCallback.onStep(System.currentTimeMillis());
                        Log.d("STEPD", "Step!");
                    }
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

    @Override
    public void start() {
        started = true;
    }

    @Override
    public void stop() {
        started = false;
    }
}
