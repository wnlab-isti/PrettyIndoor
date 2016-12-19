package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IKalmanFilter;

public abstract class KalmanFilterPredictor {
    protected IKalmanFilter filter;

    protected KalmanFilterPredictor(IKalmanFilter filter) {
        this.filter = filter;
    }
}
