package it.cnr.isti.wnlab.indoornavigator.androidutils.compass;

import android.hardware.SensorManager;
import android.os.Handler;

import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.DataEmitter;
import it.cnr.isti.wnlab.indoornavigator.framework.StoppableStartable;
import it.cnr.isti.wnlab.indoornavigator.framework.types.Acceleration;
import it.cnr.isti.wnlab.indoornavigator.framework.types.Rotation;
import it.cnr.isti.wnlab.indoornavigator.framework.types.MagneticField;
import it.cnr.isti.wnlab.indoornavigator.framework.util.HeadingChangeCallback;

/**
 * Refers to http://plaw.info/2012/03/android-sensor-fusion-tutorial/
 */

public class LawitzkiCompass implements Compass {

    // Compass configuration
    private final int mRate;
    private static final float FILTER_COEFFICIENT = 0.98f;
    private HeadingChangeCallback mCallback;

    // angular speeds from gyro
    private Rotation gyro;

    // rotation matrix from gyro data
    private float[] gyroMatrix = new float[9];

    // orientation angles from gyro matrix
    private float[] gyroOrientation = new float[3];

    // mf
    private MagneticField magnet;

    // accelerometer vector
    private Acceleration accel;

    // orientation angles from accel and magnet
    private float[] accMagOrientation = new float[3];

    // accelerometer and magnetometer based rotation matrix
    private float[] rotationMatrix = new float[9];

    // For android multithreading
    private Handler mHandler;

    // PositionFilter2D's timer
    private Timer mTimer;

    public LawitzkiCompass(HeadingChangeCallback callback,
                           DataEmitter<Acceleration> accelerometer,
                           DataEmitter<Rotation> gyroscope,
                           DataEmitter<MagneticField> magnetometer,
                           int rate
    ) {
        // Rate for updating
        mRate = rate;

        gyroOrientation[0] = 0.0f;
        gyroOrientation[1] = 0.0f;
        gyroOrientation[2] = 0.0f;

        // initialise gyroMatrix with identity matrix
        gyroMatrix[0] = 1.0f; gyroMatrix[1] = 0.0f; gyroMatrix[2] = 0.0f;
        gyroMatrix[3] = 0.0f; gyroMatrix[4] = 1.0f; gyroMatrix[5] = 0.0f;
        gyroMatrix[6] = 0.0f; gyroMatrix[7] = 0.0f; gyroMatrix[8] = 1.0f;

        // Attach to sources
        accelerometer.register(new DataObserver<Acceleration>() {
            @Override
            public void notify(Acceleration data) {
                onAccelerometer(data);
            }
        });
        gyroscope.register(new DataObserver<Rotation>() {
            @Override
            public void notify(Rotation data) {
                onGyroscope(data);
            }
        });
        magnetometer.register(new DataObserver<MagneticField>() {
            @Override
            public void notify(MagneticField data) {
                onMagnetometer(data);
            }
        });

        // Callback to call for updating
        mCallback = callback;
    }

    @Override
    public void start() {
        // wait for one second until gyroscope and magnetometer/accelerometer
        // data is initialised then schedule the complementary filter task
        mTimer = new Timer();
        mHandler = new Handler();
        mTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        calculateFusedOrientation();
                    }
                });
            }
        }, 1000, mRate);
    }

    @Override
    public void stop() {
        mTimer.cancel();
    }

    /* ***************************************
     * ACCELEROMETER
     * ***************************************/

    private void onAccelerometer(Acceleration data) {
        // copy new accelerometer data into accel array
        // then calculate new orientation
        accel = data;
        calculateAccMagOrientation();
    }

    /* ***************************************
     * GYROSCOPE
     * ***************************************/

    private void onGyroscope(Rotation data) {
        gyroFunction(data);
    }

    private static void getRotationVectorFromGyro(float[] gyroValues, float[] deltaRotationVector, float timeFactor) {

        float[] normValues = new float[3];

        // Calculate the angular speed of the sample
        float omegaMagnitude =
                (float)Math.sqrt(
                        gyroValues[0] * gyroValues[0] +
                        gyroValues[1] * gyroValues[1] +
                        gyroValues[2] * gyroValues[2]);

        // Normalize the rotation vector if it's big enough to get the axis
        final float EPSILON = 0.000000001f;
        if(omegaMagnitude > EPSILON) {
            normValues[0] = gyroValues[0] / omegaMagnitude;
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }

        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;
    }

    private static final float NS2S = 1.0f / 1000000000.0f;
    private long timestamp;
    private boolean initState = true;

    public void gyroFunction(Rotation data) {
        // don't start until first accelerometer/magnetometer orientation has been acquired
        if (accMagOrientation == null)
            return;

        // initialisation of the gyroscope based rotation matrix
        if(initState) {
            float[] initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;
        }

        // copy the new gyro values into the gyro array
        // convert the raw gyro data into a rotation vector
        float[] deltaVector = new float[4];
        if(timestamp != 0) {
            final float dT = (data.timestamp - timestamp) * NS2S;
            gyro = data;
            getRotationVectorFromGyro(gyro.array, deltaVector, dT / 2.0f);
        }

        // measurement done, save current time for next interval
        timestamp = data.timestamp;

        // convert rotation vector into rotation matrix
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

        // apply the new rotation interval on the gyroscope based rotation matrix
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);

        // get the gyroscope based orientation from the rotation matrix
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
    }

    /* ***************************************
     * MAGNETOMETER
     * ***************************************/

    private void onMagnetometer(MagneticField data) {
        magnet = data;
    }

    /* ***************************************
     * FUSION
     * ***************************************/

    private void calculateAccMagOrientation() {
        if(SensorManager.getRotationMatrix(rotationMatrix, null, accel.array, magnet.array)) {
            SensorManager.getOrientation(rotationMatrix, accMagOrientation);
        }
    }

    private void calculateFusedOrientation() {
        // final orientation angles from sensor fusion
        float[] fusedOrientation = new float[3];

        float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;
        fusedOrientation[0] =
                FILTER_COEFFICIENT * gyroOrientation[0]
                        + oneMinusCoeff * accMagOrientation[0];

        fusedOrientation[1] =
                FILTER_COEFFICIENT * gyroOrientation[1]
                        + oneMinusCoeff * accMagOrientation[1];

        fusedOrientation[2] =
                FILTER_COEFFICIENT * gyroOrientation[2]
                        + oneMinusCoeff * accMagOrientation[2];

        // overwrite gyro matrix and orientation with fused orientation
        // to comensate gyro drift
        gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
        System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);

        // Update heading
        mCallback.onHeadingChange(fusedOrientation[0], timestamp);
    }

    // Utils

    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

}
