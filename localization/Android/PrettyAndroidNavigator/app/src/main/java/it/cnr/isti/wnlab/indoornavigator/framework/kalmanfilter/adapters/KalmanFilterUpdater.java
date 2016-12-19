package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IKalmanFilter;

public abstract class KalmanFilterUpdater {
    protected IKalmanFilter filter;

    protected KalmanFilterUpdater(IKalmanFilter filter) {
        this.filter = filter;
    }
}
