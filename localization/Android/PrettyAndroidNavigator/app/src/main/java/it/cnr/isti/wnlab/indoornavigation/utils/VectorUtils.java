package it.cnr.isti.wnlab.indoornavigation.utils;

public final class VectorUtils {

    private static final String LENGTH_ERROR = "v1 and v2 must be same-length";

    private VectorUtils() {}

    public static float[] addVectors(float[] v1, float[] v2) throws NullPointerException {
        if(v1.length != v2.length)
            throw new NullPointerException(LENGTH_ERROR);

        float[] result = new float[v1.length];
        for(int i = 0; i < v1.length; i++)
            result[i] = v1[i] + v2[i];

        return result;
    }

    public static float[] subVectors(float[] v1, float[] v2) throws NullPointerException {
        if(v1.length != v2.length)
            throw new NullPointerException(LENGTH_ERROR);

        float[] result = new float[v1.length];
        for(int i = 0; i < v1.length; i++)
            result[i] = v1[i] - v2[i];

        return result;
    }

}