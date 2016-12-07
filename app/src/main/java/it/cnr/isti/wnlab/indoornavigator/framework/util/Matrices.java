package it.cnr.isti.wnlab.indoornavigator.framework.util;


public class Matrices {

    private Matrices() {}

    public static float[][] multiplyMatrixMatrix(float[][] m1, float[][] m2) {
        int n = m1.length;
        float[][] res = new float[n][n];
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                res[i][j] = multiplyVectorVector(m1[i], m2[j]);
        return res;
    }

    public static float[] multiplyMatrixVector(float[][] m, float[] v) {
        int n = v.length;
        float[] res = new float[n];
        for(int i = 0; i < n; i++)
            res[i] += multiplyVectorVector(m[i], v);
        return res;
    }

    public static float multiplyVectorVector(float[] v1, float[] v2) {
        if(v1.length == v2.length) {
            float res = 0;
            int n = v1.length;
            for(int i = 0; i < n; i++)
                res += v1[i]*v2[i];
            return res;
        } else
            throw new RuntimeException("Vectors length must be the same.");
    }

    public static void transpose(float[][] m) {
        int n = m.length;
        float temp;
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++) {
                temp = m[i][j];
                m[i][j] = m[j][i];
                m[j][i] = temp;
            }
    }

    public static void add(float[][] m1, float[][] m2) {
        int n = m1.length;
        for(int i = 0; i < n; i++)
            for(int j = 0; j < n; j++)
                m1[i][j] += m2[i][j];
    }
}