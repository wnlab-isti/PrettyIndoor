package it.cnr.isti.wnlab.indoornavigator.androidutils.compass;

import android.util.Log;

import java.util.ArrayList;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.Emitter;
import it.cnr.isti.wnlab.indoornavigator.framework.callbacks.HeadingChangeCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.types.AngularVelocity;


public class SimpleGyroCompass implements Compass, DataObserver<AngularVelocity> {

    // Callback for signaling an heading change
    private HeadingChangeCallback mCallback;

    // Gyroscope data emitter
    private Emitter<AngularVelocity> mGyroscope;

    // Current heading (starts from N)
    private float heading;

    // Window test
    private ArrayList<Float> window;
    private static final int WINDOW_SIZE = 10;

    // StartableStoppable stuff
    private boolean isStarted = false;

    public SimpleGyroCompass (
            HeadingChangeCallback callback,
            Emitter<AngularVelocity> gyroscope) {
        mCallback = callback;
        mGyroscope = gyroscope;
        window = new ArrayList<Float>(WINDOW_SIZE);
    }

    // Because speed is rad/s
    private static final float NS2S = 1.0f / 1000000000.0f;
    // Final rotation vector
    //private final float[] deltaRotationVector = new float[4];
    // Last measurement's timestamp
    private float timestamp;
    // Epsilon for magnitude check
    private static final float EPSILON = 0.000000001f;

    @Override
    public void notify(AngularVelocity v) {
        Log.d("SGYROCOMPASS", "Accuracy: " + v.accuracy);
        updateHeading(v);
        if(window.size() == WINDOW_SIZE) {
            float averageHeading = 0.f;
            for(float h : window) {
                averageHeading += h;
            }
            averageHeading /= WINDOW_SIZE;
            mCallback.onHeadingChange(averageHeading, v.timestamp);
            window.remove(0);
        }
        window.add(heading);
    }

    private void updateHeading(AngularVelocity v) {
        final float dT = (v.timestamp - timestamp) * NS2S;

        // Axis of the rotation sample, not normalized yet.
        float axisX = v.x;
        float axisY = v.y;
        float axisZ = v.z;

        // Calculate the angular speed of the sample
        float omegaMagnitude = (float) Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);

        // Normalize the rotation vector if it's big enough to get the axis
        // (that is, EPSILON should represent your maximum allowable margin of error)
        if (omegaMagnitude > EPSILON) {
            axisX /= omegaMagnitude;
            axisY /= omegaMagnitude;
            axisZ /= omegaMagnitude;
        }

        float thetaOverTwo = omegaMagnitude * dT / 2.0f;
        float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);

        // -------- END OF ANDROID EXAMPLE

        // New heading for Z-axis
        float rotationZ = sinThetaOverTwo * axisZ;
        // Update heading member
        heading = (heading+rotationZ) % 360.f;

        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        /*float thetaOverTwo = omegaMagnitude * dT / 2.0f;
        float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * axisX;
        deltaRotationVector[1] = sinThetaOverTwo * axisY;
        deltaRotationVector[2] = sinThetaOverTwo * axisZ;
        deltaRotationVector[3] = cosThetaOverTwo;*/
    }

    @Override
    public void start() {
        if(!isStarted) {
            heading = 0.f;
            timestamp = System.currentTimeMillis();
            isStarted = true;
            mGyroscope.register(this);
        }
    }

    @Override
    public void stop() {
        if(isStarted) {
            isStarted = false;
            mGyroscope.unregister(this);
        }
    }

    @Override
    public boolean isStarted() {
        return isStarted;
    }
}
