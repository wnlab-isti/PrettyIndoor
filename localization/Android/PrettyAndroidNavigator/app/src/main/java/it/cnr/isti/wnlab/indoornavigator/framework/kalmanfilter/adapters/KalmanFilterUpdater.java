package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.KalmanFilter;

public abstract class KalmanFilterUpdater {
    protected KalmanFilter filter;
    protected float[][] mH;
    protected float[][] mR;


    protected KalmanFilterUpdater(KalmanFilter filter, int n) {
        this.filter = filter;
        initH(n);
        initR(n);
    }

    protected abstract void initH(int n);

    protected abstract void initR(int n);
}
