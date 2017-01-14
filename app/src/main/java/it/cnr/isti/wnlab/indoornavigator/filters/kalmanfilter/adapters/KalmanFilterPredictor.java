package it.cnr.isti.wnlab.indoornavigator.filters.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.filters.kalmanfilter.IKalmanFilter;

public abstract class KalmanFilterPredictor {
    protected IKalmanFilter filter;

    protected KalmanFilterPredictor(IKalmanFilter filter) {
        this.filter = filter;
    }
}
