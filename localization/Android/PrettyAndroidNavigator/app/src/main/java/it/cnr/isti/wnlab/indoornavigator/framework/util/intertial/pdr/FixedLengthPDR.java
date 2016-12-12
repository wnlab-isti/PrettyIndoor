package it.cnr.isti.wnlab.indoornavigator.framework.util.intertial.pdr;

import it.cnr.isti.wnlab.indoornavigator.framework.callbacks.HeadingChangeCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.callbacks.StepDetectedCallback;

/**
 * PDR utility methods.
 */
public class FixedLengthPDR extends PDR implements HeadingChangeCallback, StepDetectedCallback {

    // Assume fixed-length steps
    private final float STEP_LENGTH = 0.6f;

    // Heading
    private float mHeading;

    public FixedLengthPDR(float initialHeading) {
        // Initialize heading
        mHeading = initialHeading;
    }

    /**
     * Update heading when possible.
     * @param newHeading In RADIANTS.
     */
    @Override
    public void onHeadingChange(float newHeading, long timestamp) {
        mHeading = newHeading;
    }

    /**
     * For each step, make a KF prediction from PDR and notify new position.
     * @param timestamp
     */
    @Override
    public void onStep(long timestamp) {
        notifyObservers(
                new PDR.Result(
                        -(STEP_LENGTH * (float) Math.sin(mHeading)),
                        STEP_LENGTH * (float) Math.cos(mHeading),
                        mHeading,
                        timestamp));
    }

}
