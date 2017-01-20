package it.cnr.isti.wnlab.indoornavigation.filters.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigation.filters.kalmanfilter.KalmanFilter;

public abstract class KalmanFilterUpdater {
    protected KalmanFilter filter;

    protected KalmanFilterUpdater(KalmanFilter filter) {
        this.filter = filter;
    }
}
