package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.kalmanfilter;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.MatrixUtils;

public class FixedStepKalmanFilter extends NewIndoorKalmanFilter {

    float startHeading;
    float stepLength;

    public FixedStepKalmanFilter(XYPosition initialPosition, float startHeading, float stepLength) {
        super(initialPosition, initX(initialPosition, startHeading), initMatrixP());
        this.startHeading = startHeading;
        this.stepLength = stepLength;
    }

    /**
     * @param position
     * @param startHeading
     * @return new [x,y,startHeading,stepLength]
     */
    protected static float[] initX(XYPosition position, float startHeading) {
        float[] x = new float[3];
        x[0] = position.x;
        x[1] = position.y;
        x[2] = startHeading;
        return x;
    }

    protected static float[][] initMatrixP() {
        float[][] mP = MatrixUtils.newIdentity(3);
        return mP;
    }

    @Override
    protected float[][] initMatrixA() {
        float[][] mA = new float[][] {
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        };
        return mA;
    }

    @Override
    protected float[][] initMatrixB() {
        float[][] mB = new float[][] {
                {0, 0},
                {0, 0},
                {0, 1} // dT (will be multiplied with Heading noise)
        };
        return mB;
    }

    @Override
    protected float[][] initMatrixQ() {
        float[][] mQ = new float[][] {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}
        };
        return mQ;
    }

    @Override
    protected float[][] initMatrixH() {
        float[][] mH = new float[][] {
                {1, 0, 0},
                {0, 1, 0}
        };
        return mH;
    }

    @Override
    protected float[][] initMatrixR() {
        float[][] mR = new float[][] {
                {1, 0},
                {0, 1}
        };
        return mR;
    }

    @Override
    protected float[][] newTransposedA(float[][] mA) {
        return initMatrixA(); // identity
    }

    @Override
    protected float[][] newTransposedH(float[][] mH) {
        float[][] mHt = new float[][] {
                {1, 0},
                {0, 1},
                {0, 0}
        };
        return mHt;
    }

    @Override
    protected float[] statePrediction(float[][] mA, float[] x, float[][] mB, float[] u) {
        return new float[0];
    }

    @Override
    protected float[][] covariancePrediction(float[][] mA, float[][] mP, float[][] mAt, float[][] mQ) {
        return new float[0][];
    }

    @Override
    protected XYPosition getPositionFromState(float[] x) {
        return new XYPosition(x[0],x[1]);
    }

    @Override
    protected float[] innovation(float[] z, float[][] mH, float[] x) {
        return new float[0];
    }

    @Override
    protected float[][] kalmanGain(float[][] mP, float[][] mHt, float[][] mSinv) {
        return new float[0][];
    }

    @Override
    protected float[][] innovationCovariance(float[][] mH, float[][] mP, float[][] mHt, float[][] mR) {
        return new float[0][];
    }

    @Override
    protected void invertInnovationCovariance(float[][] mS) {

    }

    @Override
    protected float[] updateState(float[] x, float[][] mK, float[] y) {
        return new float[0];
    }

    @Override
    protected float[][] updateCovariance(float[][] mP, float[][] mK, float[][] mH) {
        return new float[0][];
    }
}
