package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IKalmanFilter;

public abstract class KalmanFilterUpdater {
    protected IKalmanFilter filter;
    protected float[][] mH;
    protected float[][] mR;


    protected KalmanFilterUpdater(IKalmanFilter filter) {
        this.filter = filter;
        mH = initH();
        mR = initR();
    }

    protected abstract float[][] initH();

    protected abstract float[][] initR();
}
