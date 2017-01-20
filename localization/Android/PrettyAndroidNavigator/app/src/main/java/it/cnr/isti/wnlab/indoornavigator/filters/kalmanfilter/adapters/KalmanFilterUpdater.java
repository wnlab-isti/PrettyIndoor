package it.cnr.isti.wnlab.indoornavigator.filters.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.filters.kalmanfilter.KalmanFilter;

public abstract class KalmanFilterUpdater {
    protected KalmanFilter filter;

    protected KalmanFilterUpdater(KalmanFilter filter) {
        this.filter = filter;
    }
}
