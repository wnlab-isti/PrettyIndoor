package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IndoorKalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.KalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.util.intertial.pdr.PDR;

/**
 * Kalman Filter wrapper for prediction with PDR.
 * Adapts PDR's (dN,dE) to a prediction matrix for KF and triggers prediction.
 */
public class PDRPredictor extends KalmanFilterPredictor {

    public PDRPredictor(KalmanFilter filter) {
        super(filter);
    }

    @Override
    protected float[][] initA() {
        int n = IndoorKalmanFilter.N;
        float[][] mA = new float[n][n];

        // Initialize A
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                mA[i][j] = (i == j ? 1.f : 0.f);

        return mA;
    }

    @Override
    protected float[][] initQ() {
        int n = IndoorKalmanFilter.N;
        float[][] mQ = new float[n][n];

        // Initialize Q as zero matrix
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                mQ[i][j] = 0.f;

        return mQ;
    }

    /**
     * Make prediction from PDR.
     */
    public void predict(PDR.Result result) {
        float heading = result.heading;
        float sinH = (float) Math.sin(heading);
        float cosH = (float) Math.cos(heading);

        // Transformation matrix: composed stepLength translation and heading rotation
        mA[0][2] = result.dN; mA[0][3] = cosH;
        mA[1][2] = result.dE; mA[1][3] = sinH;

        // Filter's prediction
        filter.predict(mA,mQ);
    }

}
