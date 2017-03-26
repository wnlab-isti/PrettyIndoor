package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization;

import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.kalmanfilter.AbstractKalmanFilter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.VectorUtils;

class BazookaKalmanFilter extends AbstractKalmanFilter {

    public BazookaKalmanFilter() {
        super(initX(), initMatrixP());
    }

    /**
     * @return new [0,0]
     */
    protected static float[] initX() {
        float[] x = new float[2];
        x[0] = 0;
        x[1] = 0;
        return x;
    }

    protected static float[][] initMatrixP() {
        float[][] mP = newIdentity(2);
        return mP;
    }

    @Override
    protected float[][] initMatrixA() {
        float[][] mA = new float[][] {
                {1, 0},
                {0, 1}
        };
        return mA;
    }

    @Override
    protected float[][] initMatrixB() {
        float[][] mB = new float[][] {
                {0, 0},
                {0, 0}
        };
        return mB;
    }

    @Override
    protected float[][] initMatrixQ() {
        float[][] mQ = new float[][] {
                {0, 0},
                {0, 0}
        };
        return mQ;
    }

    @Override
    protected float[][] initMatrixH() {
        float[][] mH = new float[][] {
                {1, 0},
                {0, 1}
        };
        return mH;
    }

    @Override
    protected float[][] initMatrixR() {
        float[][] mR = new float[][] {
                {0, 0},
                {0, 0}
        };
        return mR;
    }

    @Override
    protected float[][] newTransposedA(float[][] mA) {
        return initMatrixA(); // unit matrix
    }

    @Override
    protected float[][] newTransposedH(float[][] mH) {
        float[][] mHt = new float[][] {
                {1, 0},
                {0, 1}
        };
        return mHt;
    }

    @Override
    protected float[] statePrediction(float[][] mA, float[] x, float[][] mB, float[] u) {
        return VectorUtils.subVectors(multiplyMatrix2x2ToVector(mA,x), multiplyMatrix2x2ToVector(mB,u));
    }

    @Override
    protected float[][] covariancePrediction(float[][] mA, float[][] mP, float[][] mAt, float[][] mQ) {
        return add2x2(multiply2x2(multiply2x2(mA,mP),mAt),mQ);
    }

    @Override
    protected float[] innovation(float[] z, float[][] mH, float[] x) {
        return VectorUtils.subVectors(z, multiplyMatrix2x2ToVector(mH,x));
    }

    @Override
    protected float[][] innovationCovariance(float[][] mH, float[][] mP, float[][] mHt, float[][] mR) {
        return add2x2(multiply2x2(multiply2x2(mH,mP),mHt),mR);
    }

    @Override
    protected float[][] kalmanGain(float[][] mP, float[][] mHt, float[][] mSinv) {
        return multiply2x2(multiply2x2(mP, mHt), mSinv);
    }

    @Override
    protected void invertInnovationCovariance(float[][] mS) {
        // Do nothing: S is unit matrix
    }

    @Override
    protected float[] updateState(float[] x, float[][] mK, float[] y) {
        return VectorUtils.addVectors(x,multiplyMatrix2x2ToVector(mK,y));
    }

    @Override
    protected float[][] updateCovariance(float[][] mP, float[][] mK, float[][] mH) {
        return multiply2x2(sub2x2(newIdentity(2),multiply2x2(mK,mH)),mP);
    }

    /*
     * 2x2 matrix utilities
     */

    private static float[][] newIdentity(int n) {
        return newIdentity(n,n);
    }

    private static float[][] newIdentity(int r, int c) {
        float[][] matrix = new float[r][c];
        for(int i = 0; i < r; i++)
            for(int j = 0; j < c; j++)
                matrix[i][j] = (i == j ? 1 : 0);
        return matrix;
    }

    /*
     * Vectors utilities
     */

    private static float[][] add2x2(float[][] mA, float[][] mB) {
        float[][] mR = new float[2][2];
        mR[0][0] = mA[0][0] + mB[0][0]; mR[0][1] = mA[0][1] + mB[0][1];
        mR[1][0] = mA[1][0] + mB[1][0]; mR[1][1] = mA[1][1] + mB[1][1];
        return mR;
    }

    private static float[][] sub2x2(float[][] mA, float[][] mB) {
        float[][] mR = new float[2][2];
        mR[0][0] = mA[0][0] - mB[0][0]; mR[0][1] = mA[0][1] - mB[0][1];
        mR[1][0] = mA[1][0] - mB[1][0]; mR[1][1] = mA[1][1] - mB[1][1];
        return mR;
    }

    private static float[][] multiply2x2(float[][] mA, float[][] mB) {
        float[][] mR = new float[2][2];
        mR[0][0] = mA[0][0] * mB[0][0] + mA[0][1] * mB[1][0];
        mR[0][1] = mA[0][0] * mB[0][1] + mA[0][1] * mB[1][1];
        mR[1][0] = mA[1][0] * mB[0][0] + mA[1][1] * mB[1][0];
        mR[1][1] = mA[1][0] * mB[0][1] + mA[1][1] * mB[1][1];
        return mR;
    }

    private static float[] multiplyMatrix2x2ToVector(float[][] matrix, float[] vector) {
        float[] result = new float[2];
        result[0] = matrix[0][0] * vector[0] + matrix[0][1] * vector[1];
        result[1] = matrix[1][0] * vector[0] + matrix[1][1] * vector[1];
        return result;
    }

}
