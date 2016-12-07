package it.cnr.isti.wnlab.indoornavigator.framework.util.kalmanfilter;

/**
 * Manages model-specific matrices and vectors and wraps the filter object.
 * Contains and initializes A and Q matrices.
 */
public abstract class KalmanFilterPredictionAdapter {
    protected KalmanFilter filter;
    protected float[][] mA;
    protected float[][] mQ;


    protected KalmanFilterPredictionAdapter(KalmanFilter filter, int n) {
        this.filter = filter;
        mA = new float[n][n];
        initA(mA);
        mQ = new float[n][n];
        initQ(mQ);
    }

    /**
     * Initialize as identity matrix.
     * @param mA A matrix.
     */
    protected void initA(float[][] mA) {
        for(int i = 0; i < mA.length; i++)
            for(int j = 0; j < mA.length; j++)
                mA[i][j] = (i == j ? 1.f : 0.f);
    }

    /**
     * Initialize as zero matrix.
     * @param mQ Q matrix.
     */
    protected void initQ(float[][] mQ) {
        int n = mQ.length;
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                mQ[i][j] = 0.f;
    }
}
