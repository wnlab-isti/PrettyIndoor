package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IKalmanFilter;

/**
 * Manages model-specific matrices and vectors and wraps the filter object.
 * Contains and initializes A and Q matrices.
 */
public abstract class KalmanFilterPredictor {
    protected IKalmanFilter filter;
    protected float[][] mA;
    protected float[][] mQ;


    protected KalmanFilterPredictor(IKalmanFilter filter) {
        this.filter = filter;
        mA = initA();
        mQ = initQ();
    }

    /**
     * Initialize as identity matrix.
     * @return Matrix A.
     */
    protected abstract float[][] initA();

    /**
     * Initialize as zero matrix.
     * @return Matrix Q.
     */
    protected abstract float[][] initQ();
}
