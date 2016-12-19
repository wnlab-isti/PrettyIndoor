package it.cnr.isti.wnlab.indoornavigator.framework.util;


public final class Vectors {

    /**
     * Subtracts v2's elements from v1's elements.
     * @param v1 The modified vector
     * @param v2
     */
    public static final void subtract(float[] v1, float[] v2) {
        for(int i = 0; i < v1.length; i++)
            v1[i] -= v2[i];
    }

    /**
     * Adds v2's elements from v1's elements.
     * @param v1 The modified vector
     * @param v2
     */
    public static final void add(float[] v1, float[] v2) {
        for(int i = 0; i < v1.length; i++)
            v1[i] += v2[i];
    }

}
