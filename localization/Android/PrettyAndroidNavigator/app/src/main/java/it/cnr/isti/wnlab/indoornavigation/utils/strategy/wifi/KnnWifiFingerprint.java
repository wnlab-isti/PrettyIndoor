package it.cnr.isti.wnlab.indoornavigation.utils.strategy.wifi;

import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;

import it.cnr.isti.wnlab.indoornavigation.observers.DataObserver;
import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.AbstractLocationStrategy;
import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.SingleAccessPoint;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.WifiFingerprint;


public class KnnWifiFingerprint
        extends AbstractLocationStrategy
        implements DataObserver<WifiFingerprint> {

    // Integer: BSSD code, Set: { (pos1, rss1), (pos2, rss2), ... }
    private HashMap<String, HashSet<PositionRss>> mBssdPositionRssAssociation;

    // Calculated distance from vector for each interesting row
    private int[] mRowDistances;

    // Possible positions
    private XYPosition[] mPositions;

    // Maximum threshold for distance: if position's row distance is over this, it isn't accepted
    private int mThreshold;

    // Selected floor. Note it's constant, so it's needed one KnnWifiFingerprint per floor
    private final int FLOOR;

    // "No/low signal" constant
    public static final int NO_SIGNAL = -100;

    /**
     * (positionIndex, measured RSSI)
     */
    private static class PositionRss {
        public final int positionIndex;
        public final int rssi;

        public PositionRss(int positionIndex, int rssi) {
            this.positionIndex = positionIndex;
            this.rssi = rssi;
        }
    }

    /**
     * Custom position subtype.
     */
    private class WifiFingerprintPosition extends IndoorPosition {

        public WifiFingerprintPosition(XYPosition p, int floor, long timestamp) {
            super(p, floor, timestamp);
        }

        public String toString() {
            return "WIFI " + super.toString();
        }
    }

    /**
     * KnnWifiFingerprint constructor.
     *
     * @param sets For each BSSD, the positions set where BSSD can be found in.
     * @param positions Possible positions.
     * @param floor Selected floor.
     */
    private KnnWifiFingerprint(
            HashMap<String, HashSet<PositionRss>> sets,
            XYPosition[] positions,
            int floor,
            int threshold
    ) {
        mBssdPositionRssAssociation = sets;
        mRowDistances = new int[positions.length];
        mPositions = positions;
        FLOOR = floor;
        mThreshold = threshold;
    }

    /**
     * Callback triggered by a fingerprint reception.
     * @param data
     */
    @Override
    public void notify(WifiFingerprint data) {
        IndoorPosition p = localize(data);
        if(p != null)
            notifyObservers(p);
    }

    /**
     * Find the minimum-distanced row (position) from received fingerprint.
     * @param fingerprint Received fingerprint
     * @return index of the minimum distanced row
     */
    private WifiFingerprintPosition localize(WifiFingerprint fingerprint) {
        // Reset every distance
        Arrays.fill(mRowDistances, 0);

        // Set filled of possible positions
        HashSet<Integer> solutions = new HashSet<>();

        // For each found AP
        for(SingleAccessPoint ap : fingerprint) {

            // Get AP's possible positions
            HashSet<PositionRss> positions = mBssdPositionRssAssociation.get(ap.bssid);

            // An AP might not be measured. If we have information about it, do the math
            if(positions != null) {
                // For each position, add it to solutions set and calculate row distance contribute
                for (PositionRss pos : positions) {
                    solutions.add(pos.positionIndex);
                    int diff = pos.rssi - ap.level;
                    mRowDistances[pos.positionIndex] += diff * diff;
                }
            }
        }

        // Find minimum distanced row
        int minRow = -1;
        int minDelta = Integer.MAX_VALUE;
        for(Integer n : solutions)
            if(mRowDistances[n] < minDelta) {
                minRow = n;
                minDelta = mRowDistances[n];
            }

        // Check threshold
        if(minDelta < mThreshold)
            return new WifiFingerprintPosition(mPositions[minRow], FLOOR, fingerprint.timestamp);
        else
            return null;
    }

    /**
     * Factory method.
     *
     * The compatible format for TSV map is:
     *
     * \t\tBSSD1\tBSSD2\tBSSD3\t...\n
     * X1\tY1\tRSSI1(X1,Y1)\tRSSI2(X1,Y1)\tRSSI3(X1,Y1)...\n
     * X2\tY2\tRSSI1(X2,Y2)\tRSSI2(X2,Y2)\tRSSI3(X2,Y2)...\n
     * ...
     * Xn\tYn\tRSSI1(Xn,Yn)\tRSSI2(Xn,Yn)\tRSSI3(Xn,Yn)...
     *
     * NOTE: the table ends without '\n'
     *
     * @return A ready-to-use WiFi fingerprint locator. Null if an error occurs.
     */
    public static KnnWifiFingerprint makeInstance(File tsvWellFormattedRSSI, int floor, int threshold) {
        try(BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(tsvWellFormattedRSSI)))) {
            // Initialize map of (string, set of positions)
            HashMap<String, HashSet<PositionRss>> sets = new HashMap<>();

            // Initialize positions list
            ArrayList<XYPosition> positions = new ArrayList<>();

            // Initialize a sets list useful to have for populating the sets map
            ArrayList<HashSet<PositionRss>> setList = new ArrayList<>();

            // Parse known BSSDs from first line
            String bssdTogether = br.readLine().substring(2);
            String[] bssds = bssdTogether.split("\t");
            for(int i = 0; i < bssds.length; i++) {
                HashSet<PositionRss> set = new HashSet<>();
                sets.put(bssds[i], set);
                setList.add(set);
            }

            // For each remaining line, parse RSSIs for each position
            String r;
            int npos = 0;
            while( (r = br.readLine()) != null ) {
                // File is TSV
                String[] splitr = r.split("\t");

                // First two elements are X,Y
                positions.add(new XYPosition(Float.parseFloat(splitr[0]),Float.parseFloat(splitr[1])));

                // Other elements are values for each BSSD
                for(int i = 2; i < splitr.length; i++) {
                    int rssi = Integer.parseInt(splitr[i]);
                    if(rssi > NO_SIGNAL)
                        setList.get(i-2).add(new PositionRss(npos, rssi));
                }

                npos++;
            }

            // Return the ready-to-use wifiFingerprintLocator instance
            XYPosition[] positionArray = new XYPosition[positions.size()];
            positions.toArray(positionArray);

            // ---------------------- DEBUG
/*            Log.d("WIFILOCBUILD","Positions: " + positionArray.length);
            for(int i = 0; i < positionArray.length; i++)
                Log.d("WIFILOCBUILD",positionArray[i].toString());
            Log.d("WIFILOCBUILD","BSSDs: " + sets.size() + " ; " + setList.size());
            for(HashSet<PositionRss> set : setList)
                Log.d("WIFILOCBUILD", setList.indexOf(set) + " has #sets = " + set.size());
*/          // ====================== DEBUG

            return new KnnWifiFingerprint(sets, positionArray, floor, threshold);
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ArrayIndexOutOfBoundsException e) {
            Log.e("WifiFingLocFactory", "File is not well-formatted");
        }

        return null;
    }

}