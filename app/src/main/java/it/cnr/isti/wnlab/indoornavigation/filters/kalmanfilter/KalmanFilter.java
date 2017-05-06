package it.cnr.isti.wnlab.indoornavigation.filters.kalmanfilter;

import it.cnr.isti.wnlab.indoornavigation.filters.StateEstimationFilter;

/**
 * Kalman Filter interface.
 */
public interface KalmanFilter extends StateEstimationFilter {

    /**
     * Kalman Filter prediction (of the next state).
     * @param u : Control vector. This indicates the magnitude of any control system's or user's
     *          control on the situation.
     */
    void predict(float[] u);

    /**
     * Kalman Filter update.
     * @param z : Measurement vector. This contains the real-world measurement we received
     *           in this time step.
     */
    void update(float[] z);

    /**
     * @return Current state vector.
     */
    float[] getStateVector();

    /**
     * @return Current covariance matrix.
     */
    float[][] getCovarianceMatrix();

}
