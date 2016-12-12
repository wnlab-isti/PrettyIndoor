package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter;

import it.cnr.isti.wnlab.indoornavigator.framework.Filter;

/**
 * Kalman Filter interface without control matrix and vector (B and u).
 */
public interface KalmanFilter extends Filter {

    /**
     * Kalman Filter prediction.
     * @param mA (matrix): State transition matrix. Basically, multiply state by this and add control
     *            factors, and you get a prediction of the state for the next time step.
     * @param mQ (matrix): Estimated process error covariance.
     */
    void predict(float[][] mA, float[][] mQ);

    /**
     * Kalman Filter update.
     * @param zn (vector): Measurement vector. This contains the real-world measurement we received
     *           in this time step.

     * @param mH (matrix): Multiply a state vector by H to translate it to a measurement
     *           vector.
     * @param R (matrix): Estimated measurement error covariance.
     */
    void update(float[] zn, float[][] mH, float[][] R);

}
