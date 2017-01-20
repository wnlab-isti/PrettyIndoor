package it.cnr.isti.wnlab.indoornavigation.filters.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigation.filters.kalmanfilter.KalmanFilter;

public abstract class KalmanFilterPredictor {
    protected KalmanFilter filter;

    protected KalmanFilterPredictor(KalmanFilter filter) {
        this.filter = filter;
    }
}
