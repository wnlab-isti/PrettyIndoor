package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import android.util.Log;

import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IndoorKalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.KalmanFilter;

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
     * @param stepLength This step's length.
     * @param heading This step's heading.
     */
    public void predict(float stepLength, float heading) {
        Log.d("PDR PREDICT", "heading is " + heading);
        float sinH = (float) Math.sin(heading);
        float cosH = (float) Math.cos(heading);
        // Transformation matrix: composed stepLength translation and heading rotation
        mA[0][2] = /*0.f*/-stepLength * sinH;  mA[0][3] = cosH;
        mA[1][2] = /*0.f*/stepLength * cosH;   mA[1][3] = sinH;
        // Filter's prediction
        filter.predict(mA,mQ);
    }

}
