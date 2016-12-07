package it.cnr.isti.wnlab.indoornavigator.framework.util;

/**
 * PDR utility methods.
 */
public class PDR {
    private PDR() {} // Static class

    /**
     * @param stepLength Step length.
     * @param heading Heading during step (rad).
     * @return float[2] delta: delta[0] = dN, delta[1] = dE.
     */
    public static float[] getFromHeadingFixedStep(float stepLength, float heading) {
        float[] delta = new float[2];
        delta[0] = -(stepLength * (float) Math.sin(heading));
        delta[1] = stepLength * (float) Math.cos(heading);
        return delta;
    }
}
