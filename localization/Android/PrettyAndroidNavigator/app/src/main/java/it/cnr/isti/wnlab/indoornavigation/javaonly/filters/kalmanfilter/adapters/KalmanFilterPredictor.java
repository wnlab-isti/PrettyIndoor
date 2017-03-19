package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.kalmanfilter.KalmanFilter;

public abstract class KalmanFilterPredictor {
    protected KalmanFilter filter;

    protected KalmanFilterPredictor(KalmanFilter filter) {
        this.filter = filter;
    }
}
