package it.cnr.isti.wnlab.indoornavigator.framework.util.strategy;

import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.LocationStrategy;
import it.cnr.isti.wnlab.indoornavigator.framework.PositionUpdateCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.util.HeadingChangeCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.util.StepDetectedCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.util.kalmanfilter.IndoorKalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.util.kalmanfilter.PDRPredictor;

/**
 * Implementation of strategy used by You Li.
 */
public class KFUleeStrategy extends LocationStrategy implements HeadingChangeCallback, StepDetectedCallback {

    // Assume fixed-length steps
    private final float STEP_LENGTH = 0.6f;

    // Heading
    private float mHeading;

    // Kalman Filter
    private IndoorKalmanFilter kf;
    private PDRPredictor pdr2kf;
    // private WifiFingerprintUpdater wifi2kf;
    // private MagneticMismatchUpdater mm2kf;

    public KFUleeStrategy(IndoorPosition startPosition, float initialHeading, PositionUpdateCallback updater) {
        // Initialize position and updater
        super(startPosition, updater);

        // Initialize heading
        mHeading = initialHeading;

        // Initialize KF
        kf = new IndoorKalmanFilter(startPosition, 5.0f, (float) Math.toRadians(10.0f), 0.5f); // with You Li's initialization
        pdr2kf = new PDRPredictor(kf);
        //wifiFinger2kf = new WifiFingerprintUpdater(kf)
        //mm2kf = new MagneticMismathUpdater(kf)
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
     * For each step, make a KF prediction from PDR.
     * @param timestamp
     */
    @Override
    public void onStep(long timestamp) {
        pdr2kf.predict(STEP_LENGTH, mHeading);
        updatePosition(kf.positionInstance(floor, timestamp));
    }

}
