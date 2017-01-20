package it.cnr.isti.wnlab.indoornavigation.filters.kalmanfilter;

import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.filters.PositionFilter2D;
import it.cnr.isti.wnlab.indoornavigation.XYPosition;

/**
 * Kalman Filter for indoor localization.
 * Initializes x vector = (x,y,1,1) and P matrix
 * with d(P) = (initPosVar, initPosVar, heading, stepLength).
 */
public class IndoorKalmanFilter implements KalmanFilter, PositionFilter2D {

    public static final int X_POSITION_IN_VECTOR = 0;
    public static final int Y_POSITION_IN_VECTOR = 1;

    private final org.apache.commons.math3.filter.KalmanFilter kf;

    private final RealMatrix matrixA;
    private final RealMatrix matrixB;
    private final RealMatrix matrixH;
    private final RealMatrix matrixQ;
    private final RealMatrix matrixR;

    public IndoorKalmanFilter(
            final double[] x0,
            double[][] mA, double[][] mB,
            double[][] mH, double[][] mQ, double[][] mR,
            final double[][] mP0
    ) {
        // Constants
        this.matrixA = MatrixUtils.createRealMatrix(mA);
        this.matrixB = MatrixUtils.createRealMatrix(mB);
        this.matrixH = MatrixUtils.createRealMatrix(mH);
        this.matrixQ = MatrixUtils.createRealMatrix(mQ);
        this.matrixR = MatrixUtils.createRealMatrix(mR);

        // Process
        ProcessModel process = new ProcessModel() {

            @Override
            public RealMatrix getStateTransitionMatrix() {
                return matrixA;
            }

            @Override
            public RealMatrix getControlMatrix() {
                return matrixB;
            }

            @Override
            public RealMatrix getProcessNoise() {
                return matrixQ;
            }

            @Override
            public RealVector getInitialStateEstimate() {
                return new ArrayRealVector(x0);
            }

            @Override
            public RealMatrix getInitialErrorCovariance() {
                return MatrixUtils.createRealMatrix(mP0);
            }
        };

        // Measurement
        MeasurementModel measure = new MeasurementModel() {
            @Override
            public RealMatrix getMeasurementMatrix() {
                return matrixH;
            }

            @Override
            public RealMatrix getMeasurementNoise() {
                return matrixR;
            }
        };

        // Initialize KF
        kf = new org.apache.commons.math3.filter.KalmanFilter(process, measure);
    }

    @Override
    public IndoorPosition getPosition(int floor, long timestamp) {
        RealVector x = kf.getStateEstimationVector();
        return new IndoorPosition(
                (float) x.getEntry(X_POSITION_IN_VECTOR),
                (float) x.getEntry(Y_POSITION_IN_VECTOR),
                floor, timestamp);
    }

    @Override
    public XYPosition get2DPosition() {
        RealVector x = kf.getStateEstimationVector();
        return new XYPosition(
                (float) x.getEntry(X_POSITION_IN_VECTOR),
                (float) x.getEntry(Y_POSITION_IN_VECTOR));
    }

    @Override
    public void predict(float[] u) {
        double[] uu = new double[u.length];
        for(int i = 0; i < u.length; i++)
            uu[i] = (double) u[i];
        kf.predict(uu);
    }

    @Override
    public void update(float[] z) {
        double[] zz = new double[z.length];
        for(int i = 0; i < z.length; i++)
            zz[i] = (double) z[i];
        kf.correct(zz);
    }
}
