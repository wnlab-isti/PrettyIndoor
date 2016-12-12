package it.cnr.isti.wnlab.indoornavigator.framework.util.strategy;

import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.LocationStrategy;
import it.cnr.isti.wnlab.indoornavigator.framework.Observer;
import it.cnr.isti.wnlab.indoornavigator.framework.callbacks.HeadingChangeCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.callbacks.StepDetectedCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IndoorKalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters.MagneticMismatchUpdater;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters.PDRPredictor;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters.WifiFingerprintUpdater;
import it.cnr.isti.wnlab.indoornavigator.framework.util.geomagnetic.mm.MagneticMismatchLocator;
import it.cnr.isti.wnlab.indoornavigator.framework.util.wifi.fingerprint.WifiFingerprintLocator;

/**
 * Implementation of strategy used by You Li.
 */
public class KFUleeStrategy extends LocationStrategy implements HeadingChangeCallback, StepDetectedCallback {

    // Assume fixed-length steps
    private final float STEP_LENGTH = 0.6f;

    // Heading
    private float mHeading;

    // Floor
    private int mFloor;

    // Kalman Filter
    private IndoorKalmanFilter kf;
    private PDRPredictor pdr2kf;
    private WifiFingerprintUpdater wifi2kf;
    private MagneticMismatchUpdater mm2kf;


    public KFUleeStrategy(
            IndoorPosition startPosition, float initialHeading, int initialFloor,
            WifiFingerprintLocator wifiLocator, MagneticMismatchLocator mmLocator) {

        // Initialize heading
        mHeading = initialHeading;

        // Initialize floor
        mFloor = initialFloor;

        // Initialize KF
        kf = new IndoorKalmanFilter(startPosition, 5.0f, (float) Math.toRadians(10.0f), 0.5f); // with You Li's initialization
        pdr2kf = new PDRPredictor(kf);
        wifi2kf = new WifiFingerprintUpdater(kf);
        mm2kf = new MagneticMismatchUpdater(kf);

        // Update KF with WiFi fingerprint positions
        if(wifiLocator != null)
            wifiLocator.register(new Observer<IndoorPosition>() {
                @Override
                public void notify(IndoorPosition wifiPosition) {
                    wifi2kf.update(wifiPosition);
                }
            });

        // Update KF with MM positions
        if(mmLocator != null)
            mmLocator.register(new Observer<IndoorPosition>() {
                @Override
                public void notify(IndoorPosition mmPosition) {
                    mm2kf.update(mmPosition);
                }
            });
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
        pdr2kf.predict(STEP_LENGTH, mHeading);
        notifyObservers(kf.positionInstance(mFloor, timestamp));
    }



}
