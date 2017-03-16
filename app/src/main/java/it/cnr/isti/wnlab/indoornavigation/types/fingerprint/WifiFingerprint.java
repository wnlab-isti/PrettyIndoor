package it.cnr.isti.wnlab.indoornavigation.types.fingerprint;

import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.SingleAccessPoint;

public class WifiFingerprint extends Fingerprint<XYPosition,AccessPoints> {

    public static final int AP_ORDER_IN_ROW = AccessPoints.ORDER_BY_BSSID_ASC;

    // Sort the measurement before calculating nearest K rows.
    @Override
    public List<XYPosition> findNearestK(AccessPoints measurement, int k) {
        // Optimize ordering
        // NO THREAD SAFE!!!!
        measurement.sort(AP_ORDER_IN_ROW);

        // Now we can the distance
        return super.findNearestK(measurement,k);
    }

    /**
     * Distance between AP scansions computed as the difference between common APs' levels.
     * Uncommon APs are ignored.
     * @param aps1
     * @param aps2
     * @return the distance between two AP scansions.
     */
    @Override
    protected float distanceBetween(AccessPoints aps1, AccessPoints aps2) {
        // Some useful variables...
        int i1 = 0;
        int l1 = aps1.size();
        SingleAccessPoint[] array1 = aps1.getArray();
        int i2 = 0;
        int l2 = aps2.size();
        SingleAccessPoint[] array2 = aps2.getArray();

        // Distance
        float distance = 0.f;

        // Compare arrays (Computer Science first year excercise)
        while(i1 < l1 && i2 < l2) {
            int comparation = array1[i1].bssid.compareTo(array2[i2].bssid);
            // Same BSSID
            if(comparation == 0) {
                float drssi = array1[i1].rssi - array2[i2].rssi;
                distance += drssi*drssi;
            }
            // Different BSSID, go on
            else if(comparation < 0)
                i1++;
            else if(comparation > 0)
                i2++;
        }

        return distance;
    }
}
