package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IndoorKalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.KalmanFilter;

public class WifiFingerprintUpdater extends KalmanFilterUpdater {

    public WifiFingerprintUpdater(KalmanFilter filter) {
        super(filter, IndoorKalmanFilter.N);
    }

    @Override
    protected void initH(int n) {
        // mH = new zero matrix TODO
        mH = new float[2][n];
        mH[0][0] = 1.f; mH[0][1] = 0.f; mH[0][2] = 0.f; mH[0][3] = 0.f;
        mH[1][0] = 0.f; mH[1][1] = 1.f; mH[1][2] = 0.f; mH[1][3] = 0.f;
    }

    @Override
    protected void initR(int n) {
        mR = new float[2][2];
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                if(i == j)
                    mR[i][j] = 1.f;
                else
                    mR[i][j] = 0.f;
    }

    public void update(IndoorPosition wifiPosition) {
        float[] z = new float[IndoorKalmanFilter.N];
        z[0] = wifiPosition.x;
        z[1] = wifiPosition.y;
        z[2] = 0.f;
        z[3] = 0.f;

        filter.update(z, mH, mR);
    }

}
