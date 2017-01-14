package it.cnr.isti.wnlab.indoornavigator.filters.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.filters.kalmanfilter.IKalmanFilter;

public abstract class KalmanFilterUpdater {
    protected IKalmanFilter filter;

    protected KalmanFilterUpdater(IKalmanFilter filter) {
        this.filter = filter;
    }
}
