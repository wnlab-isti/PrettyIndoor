package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.pdr;

import org.apache.commons.math3.distribution.NormalDistribution;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.Step;

public class StochasticStepPDR extends PDR {

    private final NormalDistribution angleDistribution;
    private final NormalDistribution speedDistribution;

    // Assume fixed-length steps
    private float mStepLength;

    // Heading
    private float mHeading;

    public StochasticStepPDR(Emitter<Heading> heading, Emitter<Step> stepDetector, float stepLength, float initialHeading, float angleStandardDeviation, float speedStandardDeviation) {
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

        // Initialize random distributions
        angleDistribution = new NormalDistribution(0, angleStandardDeviation);
        speedDistribution = new NormalDistribution(0, speedStandardDeviation);
    }

    /**
     * Update heading when possible.
     * @param newHeading in RADIANTS.
     */
    private void onHeadingChange(float newHeading) {
        mHeading = newHeading;
    }

    /**
     * For each step, calculate dN and dE considering a random factor.
     * @param step
     */
    private void onStep(Step step) {
        float stepLength = mStepLength + ((float) speedDistribution.sample());
        float heading = mHeading + ((float) angleDistribution.sample());

        PDR.Result res = new PDR.Result(
                -stepLength * (float) Math.sin(heading),
                stepLength * (float) Math.cos(heading),
                heading,
                step.timestamp);
        notifyObservers(res);
    }

}
