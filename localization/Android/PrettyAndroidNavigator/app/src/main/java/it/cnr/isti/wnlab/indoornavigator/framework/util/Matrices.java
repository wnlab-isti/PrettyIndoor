package it.cnr.isti.wnlab.indoornavigator.framework.util;


import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public final class Matrices {

    private Matrices() {}

    public static final float multiplyVectorVector(float[] v1, float[] v2) {
        if(v1.length == v2.length) {
            float res = 0;
            int n = v1.length;
            for(int i = 0; i < n; i++)
                res += v1[i]*v2[i];
            return res;
        } else
            throw new RuntimeException("Vectors length must be the same.");
    }

    public static final float[][] multiplyMatrixMatrix(float[][] m1, float[][] m2) {
        if(m1[0].length == m2.length) {
            int newrows = m1.length;
            int newcolumns = m2[0].length;
            int kmax = m2.length; // m == m1[0].length == m2.length
            float[][] res = new float[newrows][newcolumns];
            for (int i = 0; i < newrows; i++) {
                for (int j = 0; j < newcolumns; j++) {
                    res[i][j] = 0.f;
                    for (int k = 0; k < kmax; k++)
                        res[i][j] += m1[i][k] * m2[k][j];
                }
            }

            return res;
        } else
            throw new RuntimeException("m1 #columns and m2 #rows must be the same.");
    }

    public static final float[] multiplyMatrixVector(float[][] m, float[] v) {
        int n = v.length;
        if(m[0].length == n) {
            float[] res = new float[n];
            for (int i = 0; i < n; i++)
                res[i] += multiplyVectorVector(m[i], v);

            return res;
        } else
            throw new RuntimeException("Matrix columns and vector's length must be the same.");
    }

    public static final void transpose(float[][] m) {
        int n = m.length;
        float temp;
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++) {
                temp = m[i][j];
                m[i][j] = m[j][i];
                m[j][i] = temp;
            }
    }

    public static final float[][] makeTranspose(float[][] m) {
        int n = m.length;
        float[][] transposed = new float[m.length][m[0].length];
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                transposed[i][j] = m[j][i];

        return transposed;
    }

    public static final float[][] identity(int n) {
        return identity(n,n);
    }

    public static final float[][] identity(int r, int c) {
        float[][] matrix = new float[r][c];
        for(int i = 0; i < r; i++)
            for(int j = 0; j < c; j++)
                matrix[i][j] = (i == j) ? 1 : 0;
        return matrix;
    }

    public static final float[][] zero(int n) {
        return zero(n,n);
    }

    public static final float[][] zero(int r, int c) {
        float[][] matrix = new float[r][c];
        for(int i = 0; i < r; i++)
            for(int j = 0; j < c; j++)
                matrix[i][j] = 0;
        return matrix;
    }

    public static final void addM2toM1(float[][] m1, float[][] m2) {
        int n = m1.length;
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                m1[i][j] += m2[i][j];
    }

    public static final void subM2toM1(float[][] m1, float[][] m2) {
        int n = m1.length;
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                m1[i][j] -= m2[i][j];
    }

    public static final void invert(float[][] m) {
        // Apache commons math library works with double matrices
        double[][] doubled = new double[m.length][m[0].length];
        for(int i = 0; i < m.length; i++)
            for(int j = 0; j < m[0].length; j++)
                doubled[i][j] = (double) m[i][j];
        // Invert with apache commons
        RealMatrix inverse = MatrixUtils.inverse(MatrixUtils.createRealMatrix(doubled));
        // Re-cast matrix to float
        doubled = inverse.getData();
        for(int i = 0; i < m.length; i++)
            for(int j = 0; j < m.length; j++)
                m[i][j] = (float) doubled[i][j];
    }
}