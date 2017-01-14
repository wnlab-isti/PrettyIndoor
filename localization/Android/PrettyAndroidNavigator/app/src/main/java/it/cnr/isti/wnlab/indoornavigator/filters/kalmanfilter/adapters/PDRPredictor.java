package it.cnr.isti.wnlab.indoornavigator.filters.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.filters.kalmanfilter.IKalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.utils.intertial.pdr.PDR;

/**
 * Kalman Filter wrapper for prediction with PDR.
 * Adapts PDR's (dN, dE) to a prediction matrix for KF and triggers prediction.
 */
public class PDRPredictor extends KalmanFilterPredictor {

    private final float stepLength;

    public PDRPredictor(IKalmanFilter filter, float stepLength) {
        super(filter);
        this.stepLength = stepLength;
    }

    /**
     * Make prediction from PDR.
     */
    public void predict(PDR.Result result) {
        float heading = result.heading;
        float dN = stepLength * (float) Math.sin(heading);
        float dE = stepLength * (float) Math.cos(heading);

        // Filter's prediction
        float[] u = {dE, dN, 0.f, 0.f}; // [dx, dy, 0, 0]
        filter.predict(u);
    }

}
