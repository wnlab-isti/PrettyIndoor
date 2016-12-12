package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter;

import android.util.Log;

import it.cnr.isti.wnlab.indoornavigator.framework.util.Matrices;

/**
 * Contains state vector x and state average error matrix P.
 * Implements prediction and update calculations.
 */
public abstract class AbstractSimpleKalmanFilter implements KalmanFilter {

    protected float[] x;
    protected float[][] mP;

    /**
     * Create x vector and mP matrix. They will be initialized by concrete classes.
     * @param n x is n-long and P is nxn.
     */
    public AbstractSimpleKalmanFilter(int n) {
        x = new float[n];
        mP = new float[n][n];

        // Zero x vector
        for(int i = 0; i < n; i++)
            x[i] = 0.f;

        // Zero P matrix
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                mP[i][j] = 0.f;
    }

    @Override
    public void predict(float[][] mA, float[][] mQ) {
        // x(n) = A*x(n-1)
        // For now, control matrix and vector addition is avoided (B*u(n)).
        Log.d("OLD X VECTOR", x[0] + "," + x[1] + "," + x[2] + "," + x[3]);
        float[] xn = Matrices.multiplyMatrixVector(mA,x);
        x = xn;
        Log.d("NEW X VECTOR", x[0] + "," + x[1] + "," + x[2] + "," + x[3]);

        // P(n) = A*P(n-1)*A' + Q
        float[][] mAt = mA.clone();
        Matrices.transpose(mAt);
        mP = Matrices.multiplyMatrixMatrix(Matrices.multiplyMatrixMatrix(mA,mP),mAt);
        Matrices.add(mP, mQ);
    }

    @Override
    public void update(float[] z, float[][] mH, float[][] mR) {
        // TODO later...
    }
}
