package it.cnr.isti.wnlab.indoornavigation.utils.pdr;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.pdr.PDR;
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

    // Emitters and observers
    private Emitter<Heading> mHeadingEmitter;
    private Observer<Heading> mHeadingObserver;
    private Emitter<Step> mStepDetector;
    private Observer<Step> mStepObserver;

    public FixedStepPDR(Emitter<Heading> heading, Emitter<Step> stepDetector, float stepLength, float initialHeading) {
        // Initialize step length and heading
        mStepLength = stepLength;
        mHeading = initialHeading;

        // Heading changes
        mHeadingEmitter = heading;
        mHeadingObserver = new Observer<Heading>() {
            @Override
            public void notify(Heading data) {
                onHeadingChange(data.heading);
            }
        };

        // Step detection
        mStepDetector = stepDetector;
        mStepObserver = new Observer<Step>() {
            @Override
            public void notify(Step step) {
                onStep(step);
            }
        };
    }

    @Override
    protected void startEmission() {
        mHeadingEmitter.register(mHeadingObserver);
        mStepDetector.register(mStepObserver);
    }

    @Override
    protected void stopEmission() {
        mHeadingEmitter.unregister(mHeadingObserver);
        mStepDetector.unregister(mStepObserver);
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
