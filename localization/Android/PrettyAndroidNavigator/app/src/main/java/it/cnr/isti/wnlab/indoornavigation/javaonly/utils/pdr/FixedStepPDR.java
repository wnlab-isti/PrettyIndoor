package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.pdr;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Step;

/**
 * PDR utility methods.
 */
public class FixedStepPDR extends PDR {

    // Assume fixed-length steps
    private float mStepLength;

    // Heading
    private float mHeading;

    public FixedStepPDR(Emitter<Heading> heading, Emitter<Step> stepDetector, float stepLength, float initialHeading) {
        // Initialize step length and heading
        mStepLength = stepLength;
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
        PDR.Result res = new PDR.Result(
                -mStepLength * (float) Math.sin(mHeading),
                mStepLength * (float) Math.cos(mHeading),
                mHeading,
                step.timestamp);
        notifyObservers(res);
    }
}
