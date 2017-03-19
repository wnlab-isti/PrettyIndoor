package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.intertial.pdr;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Step;

/**
 * PDR utility methods.
 */
public class FixedLengthPDR extends PDR {

    // Assume fixed-length steps
    private final float STEP_LENGTH = 0.6f;

    // Heading
    private float mHeading;

    public FixedLengthPDR(Emitter<Heading> heading, Emitter<Step> stepDetector, float initialHeading) {
        // Initialize heading
        mHeading = initialHeading;

        // Register for heading changes
        heading.register(new Observer<Heading>() {
            @Override
            public void notify(Heading data) {
                onHeadingChange(data.heading);
            }
        });

        // Register for step detection
        stepDetector.register(new Observer<Step>() {
            @Override
            public void notify(Step step) {
                onStep(step);
            }
        });
    }

    /**
     * Update heading when possible.
     * @param newHeading in RADIANTS.
     */
    private void onHeadingChange(float newHeading) {
        mHeading = newHeading;
    }

    /**
     * For each step, make a KF prediction from PDR and notify new position.
     * @param step
     */
    private void onStep(Step step) {
        notifyObservers(
                new PDR.Result(
                        -(STEP_LENGTH * (float) Math.sin(mHeading)),
                        STEP_LENGTH * (float) Math.cos(mHeading),
                        mHeading,
                        step.timestamp));
    }
}
