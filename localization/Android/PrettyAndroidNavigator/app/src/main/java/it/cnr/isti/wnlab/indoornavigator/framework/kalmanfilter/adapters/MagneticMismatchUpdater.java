package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters;

import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IndoorKalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IKalmanFilter;

public class MagneticMismatchUpdater extends KalmanFilterUpdater {

    public MagneticMismatchUpdater(IKalmanFilter filter) {
        super(filter);
    }

    @Override
    protected float[][] initH() {
        int n = IndoorKalmanFilter.N;
        float[][] matrix = new float[2][n];

        matrix[0][0] = 1.f; matrix[0][1] = 0.f; matrix[0][2] = 0.f; matrix[0][3] = 0.f;
        matrix[1][0] = 0.f; matrix[1][1] = 1.f; matrix[1][2] = 0.f; matrix[1][3] = 0.f;

        return matrix;
    }

    @Override
    protected float[][] initR() {
        int n = 2;
        float[][] mR = new float[n][n];

        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                mR[i][j] = 0.f;

        return mR;
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
