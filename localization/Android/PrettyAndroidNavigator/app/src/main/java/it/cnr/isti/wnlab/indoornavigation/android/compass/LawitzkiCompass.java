package it.cnr.isti.wnlab.indoornavigation.android.compass;

import android.hardware.SensorManager;
import android.os.Handler;

import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigation.javaonly.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.Acceleration;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.inertial.AngularSpeed;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.environmental.MagneticField;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.MatrixUtils;

/**
 * Refers to http://plaw.info/2012/03/android-sensor-fusion-tutorial/
 */

public abstract class LawitzkiCompass extends AbstractEmitter<Heading> implements StartableStoppable {

    // Accelerometer
    private DataEmitter<Acceleration> accelerometer;
    private DataEmitter<AngularSpeed> gyroscope;
    private DataEmitter<MagneticField> magnetometer;

    // Start flag
    private boolean started = false;

    // Initial delay
    public final static int INITIAL_DELAY = 0;

    // Default rate
    public final static int DEFAULT_RATE = 60;

    // Compass configuration
    private final int mRate;

    // TODO de che è?
    private static final float FILTER_COEFFICIENT = 0.98f;

    // Angular speeds from gyro
    private AngularSpeed gyro;

    // Rotation matrix from gyro data
    private float[] gyroMatrix = new float[9];

    // Orientation angles from gyro matrix
    private float[] gyroOrientation = new float[3];

    // Magnetic field
    private MagneticField magnet;

    // Accelerometer vector
    private Acceleration accel;

    // Orientation angles from accel and magnet
    private float[] accMagOrientation = new float[3];

    // Accelerometer and magnetometer based rotation matrix
    private float[] rotationMatrix = new float[9];

    // For android multithreading
    private Handler mHandler;

    // PositionFilter2D's timer
    private Timer mTimer;

    public LawitzkiCompass(DataEmitter<Acceleration> accelerometer,
                           DataEmitter<AngularSpeed> gyroscope,
                           DataEmitter<MagneticField> magnetometer,
                           int rate
    ) {
        // Rate for updating
        mRate = rate;

        this.accelerometer = accelerometer;
        this.gyroscope = gyroscope;
        this.magnetometer = magnetometer;
    }

    public void start() {
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
        gyroscope.register(new DataObserver<AngularSpeed>() {
            @Override
            public void notify(AngularSpeed data) {
                gyroFunction(data);
            }
        });
        magnetometer.register(new DataObserver<MagneticField>() {
            @Override
            public void notify(MagneticField data) {
                onMagnetometer(data);
            }
        });

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
        }, INITIAL_DELAY, mRate);

        started = true;
    }

    public void stop() {
        mTimer.cancel();
        started = false;
    }

    @Override
    public boolean isStarted() {
        return started;
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

    private static final float NS2S = 1.0f / 1000000000.0f;
    private long timestamp;
    private boolean initState = true;

    public void gyroFunction(AngularSpeed data) {
        // don't start until first accelerometer/magnetometer orientation has been acquired
        if (accMagOrientation == null)
            return;

        // initialisation of the gyroscope based rotation matrix
        if(initState) {
            float[] initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = MatrixUtils.multiplication3x3(gyroMatrix, initMatrix);
            initState = false;
        }

        // copy the new gyro values into the gyro array
        // convert the raw gyro data into a rotation vector
        float[] deltaVector = new float[4];
        if(timestamp != 0) {
            final float dT = (data.timestamp - timestamp) * NS2S;
            gyro = data;
            getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);
        }

        // measurement done, save current time for next interval
        timestamp = data.timestamp;

        // convert rotation vector into rotation matrix
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

        // apply the new rotation interval on the gyroscope based rotation matrix
        gyroMatrix = MatrixUtils.multiplication3x3(gyroMatrix, deltaMatrix);

        // get the gyroscope based orientation from the rotation matrix
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
    }

    private static void getRotationVectorFromGyro(AngularSpeed gyroValues, float[] deltaRotationVector, float timeFactor) {

        float[] normValues = new float[3];

        // Calculate the angular speed of the sample
        float omegaMagnitude =
                (float)Math.sqrt(
                        gyroValues.x * gyroValues.x +
                                gyroValues.y * gyroValues.y +
                                gyroValues.z * gyroValues.z);

        // Normalize the rotation vector if it's big enough to get the axis
        final float EPSILON = 0.000000001f;
        if(omegaMagnitude > EPSILON) {
            normValues[0] = gyroValues.x / omegaMagnitude;
            normValues[1] = gyroValues.y / omegaMagnitude;
            normValues[2] = gyroValues.z / omegaMagnitude;
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
        if(accel != null && magnet != null && SensorManager.getRotationMatrix(rotationMatrix, null, accel.getArray(), magnet.getArray())) {
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
        // to compensate gyro drift
        gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
        System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);

        // Update heading
        onHeadingChange(fusedOrientation[0], timestamp);
    }

    /**
     * Notify observers on heading changes.
     * @param heading New heading.
     * @param timestamp Timestamp of last gyroscope measure.
     */
    protected void onHeadingChange(float heading, long timestamp) {
        notifyObservers(new Heading(heading, timestamp));
    }

    private static float[] getRotationMatrixFromOrientation(float[] orientation) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(orientation[1]);
        float cosX = (float)Math.cos(orientation[1]);
        float sinY = (float)Math.sin(orientation[2]);
        float cosY = (float)Math.cos(orientation[2]);
        float sinZ = (float)Math.sin(orientation[0]);
        float cosZ = (float)Math.cos(orientation[0]);

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
        float[] resultMatrix = MatrixUtils.multiplication3x3(xM, yM);
        resultMatrix = MatrixUtils.multiplication3x3(zM, resultMatrix);
        return resultMatrix;
    }

}
