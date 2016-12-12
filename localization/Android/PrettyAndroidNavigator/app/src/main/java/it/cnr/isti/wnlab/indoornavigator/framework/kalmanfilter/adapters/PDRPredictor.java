package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IndoorKalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.KalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.util.intertial.pdr.PDR;

/**
 * Kalman Filter wrapper for prediction with PDR.
 * Adapts PDR's (dN,dE) to a prediction matrix for KF and triggers prediction.
 */
public class PDRPredictor extends KalmanFilterPredictionAdapter {

    public PDRPredictor(KalmanFilter filter) {
        super(filter, IndoorKalmanFilter.N);
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
