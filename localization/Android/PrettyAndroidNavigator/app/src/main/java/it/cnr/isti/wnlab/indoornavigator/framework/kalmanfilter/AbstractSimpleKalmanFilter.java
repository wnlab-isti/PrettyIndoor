package it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter;

import it.cnr.isti.wnlab.indoornavigator.framework.util.Matrices;
import it.cnr.isti.wnlab.indoornavigator.framework.util.Vectors;

/**
 * Contains state vector x and state average error matrix P.
 * Implements prediction and update calculations.
 */
public abstract class AbstractSimpleKalmanFilter implements IKalmanFilter {

    protected float[] x;
    protected float[][] mP;

    /**
     * Create x vector and mP matrix. They will be initialized by concrete classes.
     */
    public AbstractSimpleKalmanFilter() {
        x = initX();
        mP = initP();
    }

    public abstract float[] initX();

    public abstract float[][] initP();

    @Override
    public void predict(float[][] mA, float[][] mQ) {
        // x(n) = A*x(n-1)
        // For now, control matrix and vector addition is avoided (B*u(n)).
        x = Matrices.multiplyMatrixVector(mA,x);

        // P(n) = A*P(n-1)*A' + Q
        float[][] mAt = mA.clone();
        Matrices.transpose(mAt);
        mP = Matrices.multiplyMatrixMatrix(Matrices.multiplyMatrixMatrix(mA,mP),mAt);
        Matrices.addM2toM1(mP, mQ);
    }

    @Override
    public void update(float[] z, float[][] mH, float[][] mR) {
        // y =  z - Hx
        Vectors.subtract(z, Matrices.multiplyMatrixVector(mH, x));
        float[] y = z;

        // S = H P H' + R
        float[][] transH = mH.clone();
        Matrices.transpose(transH);
        float[][] mS = Matrices.multiplyMatrixMatrix(Matrices.multiplyMatrixMatrix(mH, mP),transH);
        Matrices.addM2toM1(mS, mR);

        // K = P H' inv(S)
        Matrices.invert(mS);
        float[][] mK = Matrices.multiplyMatrixMatrix(Matrices.multiplyMatrixMatrix(mP, transH), mS);

        // x = x + Ky
        Vectors.add(x, Matrices.multiplyMatrixVector(mK, y));

        // P = (I - KH) P
        float[][] mKH = Matrices.multiplyMatrixMatrix(mK, mH);
        float[][] mIsubKH = Matrices.identity(mKH.length,mKH[0].length);
        Matrices.subM2toM1(mIsubKH, mKH);
        mP = Matrices.multiplyMatrixMatrix(mIsubKH, mP);
    }
}
