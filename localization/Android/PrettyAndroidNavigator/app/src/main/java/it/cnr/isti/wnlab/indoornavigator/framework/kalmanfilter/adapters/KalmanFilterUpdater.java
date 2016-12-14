package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.KalmanFilter;

public abstract class KalmanFilterUpdater {
    protected KalmanFilter filter;
    protected float[][] mH;
    protected float[][] mR;


    protected KalmanFilterUpdater(KalmanFilter filter) {
        this.filter = filter;
        mH = initH();
        mR = initR();
    }

    protected abstract float[][] initH();

    protected abstract float[][] initR();
}
