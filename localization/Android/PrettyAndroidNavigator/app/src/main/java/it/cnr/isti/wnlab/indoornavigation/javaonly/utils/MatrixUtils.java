package it.cnr.isti.wnlab.indoornavigation.javaonly.utils;

public final class MatrixUtils {

    private MatrixUtils() {}

    public static float[][] newIdentity(int n) {
        return newIdentity(n,n);
    }

    public static float[][] newIdentity(int r, int c) {
        float[][] matrix = new float[r][c];
        for(int i = 0; i < r; i++)
            for(int j = 0; j < c; j++)
                matrix[i][j] = (i == j ? 1 : 0);
        return matrix;
    }

}