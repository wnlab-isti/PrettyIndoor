package it.cnr.isti.wnlab.indoornavigation.utils.strategies.fingerprint.wifi;

import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.AbstractLocationStrategy;
import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.observer.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.types.fingerprint.WifiFingerprint;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.AccessPoints;

/**
 * KNN and euclidean average between position.
 */
public class WifiFingerprintStrategy
        extends AbstractLocationStrategy
        implements DataObserver<AccessPoints> {

    private WifiFingerprint mFingerprint;
    private int mK;

    public WifiFingerprintStrategy(WifiFingerprint fingerprint, int k) {
        mFingerprint = fingerprint;
        mK = k;
    }

    @Override
    public void notify(AccessPoints data) {
        // Sort APs in the occurred order for K-NN
        // NO THREAD SAFETY!!!!!!!!
        data.sort(WifiFingerprint.AP_ORDER_IN_ROW);

        // Do K-NN
        List<XYPosition> positions = mFingerprint.findNearestK(data, mK);

        // Find middle position
        float avgX = 0.f;
        float avgY = 0.f;
        int times = 0;
        for(XYPosition p : positions) {
            System.out.println("WifiFingerprint position: " + p);
            avgX += p.x;
            avgY += p.y;
            times++;
        }
        avgX /= times;
        avgY /= times;

        // Notify middle position
        notifyObservers(new IndoorPosition(avgX,avgY,0,System.currentTimeMillis()));
    }
}
