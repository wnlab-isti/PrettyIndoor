package it.cnr.isti.wnlab.indoornavigator.framework.util.strategy;

import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.LocationStrategy;
import it.cnr.isti.wnlab.indoornavigator.framework.Observer;
import it.cnr.isti.wnlab.indoornavigator.framework.PositionUpdateCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.XYPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.IndoorKalmanFilter;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters.MagneticMismatchUpdater;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters.PDRPredictor;
import it.cnr.isti.wnlab.indoornavigator.framework.kalmanfilter.adapters.WifiFingerprintUpdater;
import it.cnr.isti.wnlab.indoornavigator.framework.util.geomagnetic.mm.MagneticMismatchLocator;
import it.cnr.isti.wnlab.indoornavigator.framework.util.intertial.pdr.PDR;
import it.cnr.isti.wnlab.indoornavigator.framework.util.wifi.fingerprint.WifiFingerprintLocator;

/**
 * Implementation of strategy used by You Li.
 */
public class KFUleeStrategy extends LocationStrategy {

    // Kalman Filter
    private IndoorKalmanFilter kf;
    private PDRPredictor pdr2kf;
    private WifiFingerprintUpdater wifi2kf;
    private MagneticMismatchUpdater mm2kf;

    // Position update callback
    final private PositionUpdateCallback mCallback;

    // Floor
    private int mFloor;

    public KFUleeStrategy(
            IndoorPosition startPosition,
            PDR pdr,
            WifiFingerprintLocator wifiLocator,
            MagneticMismatchLocator mmLocator,
            PositionUpdateCallback callback
    ) {
        // Initialize KF
        kf = new IndoorKalmanFilter(startPosition, 5.0f, (float) Math.toRadians(10.0f), 0.5f); // with You Li's initialization
        pdr2kf = new PDRPredictor(kf);
        wifi2kf = new WifiFingerprintUpdater(kf);
        mm2kf = new MagneticMismatchUpdater(kf);

        // Floor initialization
        mFloor = startPosition.floor;

        // Callback for position updates
        mCallback = callback;

        // KF prediction with PDR
        pdr.register(new Observer<PDR.Result>() {
            @Override
            public void notify(PDR.Result pdrDelta) {
                pdr2kf.predict(pdrDelta);
                mCallback.onPositionUpdate(kf.positionInstance(mFloor,pdrDelta.timestamp));
            }
        });

        // Update KF with WiFi fingerprint positions
        if(wifiLocator != null)
            wifiLocator.register(new Observer<IndoorPosition>() {
                @Override
                public void notify(IndoorPosition wifiPosition) {
                    wifi2kf.update(wifiPosition);
                    mCallback.onPositionUpdate(kf.positionInstance(mFloor,wifiPosition.timestamp));
                }
            });

        // Update KF with MM positions
        if(mmLocator != null)
            mmLocator.register(new Observer<IndoorPosition>() {
                @Override
                public void notify(IndoorPosition mmPosition) {
                    mm2kf.update(mmPosition);
                    mCallback.onPositionUpdate(kf.positionInstance(mFloor,mmPosition.timestamp));
                }
            });
    }
}
