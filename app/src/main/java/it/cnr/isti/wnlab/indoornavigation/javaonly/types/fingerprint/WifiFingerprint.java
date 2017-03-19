package it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint;

import java.util.ArrayList;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.SingleAccessPoint;

public class WifiFingerprint extends Fingerprint<XYPosition,AccessPoints> {

    public static final int AP_ORDER_IN_ROW = AccessPoints.ORDER_BY_BSSID_ASC;
    public static final float MIN_RSSI_VALUE = -100.f;
    public static final float MAX_ROW_DISTANCE = MIN_RSSI_VALUE * MIN_RSSI_VALUE;

    private WifiFingerprint() {}

    // Sort the measurement before calculating nearest K rows.
    @Override
    public List<XYPosition> findNearestK(AccessPoints measurement, int k, float threshold) {
        // Sort needed by distance calculation
        // NO THREAD SAFETY!!!!
        measurement.sort(AP_ORDER_IN_ROW);

        // Now we can the distance
        return super.findNearestK(measurement,k,threshold);
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
            System.out.println("Comparing " + i1 + " and " + i2);
            int comparison = array1[i1].bssid.compareTo(array2[i2].bssid);
            // Same BSSID
            if(comparison == 0) {
                // Calculate distance
                float drssi = array1[i1].rssi - array2[i2].rssi;
                distance += drssi*drssi;
                // Increment both
                i1++;
                i2++;
            }
            // Different BSSID, go on
            else if(comparison < 0) {
                i1++;
                distance += MAX_ROW_DISTANCE;
            } else if(comparison > 0) {
                i2++;
                distance += MAX_ROW_DISTANCE;
            }
        }

        return distance;
    }

    /**
     * Builder class for WifiFingerprint.
     */
    public static class Builder extends FingerprintBuilder<WifiFingerprint> {

        public WifiFingerprint build(List<String> lines) {
            // Instantiate fingerprint object
            WifiFingerprint fingerprint = new WifiFingerprint();

            // Parse the text lines
            for (String l : lines) {
                // Split CSV file
                String[] splitLine = l.split(",");

                // Parse coordinate
                XYPosition position = new XYPosition(
                        Float.parseFloat(splitLine[0]),Float.parseFloat(splitLine[1]));

                // Create BSSID,RSSI instances and the wrapper object
                List<SingleAccessPoint> apList = new ArrayList<>();
                for(int i = 2; i < splitLine.length-1; i+=2)
                    apList.add(new SingleAccessPoint(splitLine[i],Integer.parseInt(splitLine[i+1])));
                AccessPoints aps = new AccessPoints(apList,System.currentTimeMillis());

                // Populate map
                fingerprint.map.put(position, aps);
            }

            // Return fingerprint instance
            return fingerprint;
        }
    }
}
