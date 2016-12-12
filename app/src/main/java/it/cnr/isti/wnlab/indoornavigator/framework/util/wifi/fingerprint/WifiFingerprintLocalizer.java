package it.cnr.isti.wnlab.indoornavigator.framework.util.wifi.fingerprint;

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

import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.LocationStrategy;
import it.cnr.isti.wnlab.indoornavigator.framework.XYPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.types.SingleAccessPoint;
import it.cnr.isti.wnlab.indoornavigator.framework.types.WifiFingerprint;


public class WifiFingerprintLocalizer extends LocationStrategy implements DataObserver<WifiFingerprint> {

    // Integer: BSSD code, Set: { (pos1, rss1), (pos2, rss2), ... }
    private HashMap<String, HashSet<PositionRss>> mBssdPositionRssAssociation;

    // Calculated distance from vector for each interesting row
    private int[] mRowDistances;

    // Possible positions
    private XYPosition[] mPositions;

    // Selected floor. Note it's constant, so it's needed one WifiFingerprintLocalizer per floor
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
     *
     * @param sets For each BSSD, the positions set where BSSD can be found in.
     * @param positions Possible positions.
     * @param floor Selected floor.
     */
    private WifiFingerprintLocalizer(
            HashMap<String, HashSet<PositionRss>> sets,
            XYPosition[] positions,
            int floor
    ) {
        // Initialize distances array
        mRowDistances = new int[sets.size()];

        // Save positions
        mPositions = positions;

        // Save selected floor
        FLOOR = floor;
    }

    /**
     * Callback triggered by a fingerprint reception.
     * @param data
     */
    @Override
    public void notify(WifiFingerprint data) {
        localize(data, FLOOR);
    }

    /**
     * Notify fingerprint localization.
     * @param fingerprint Received fingerprint.
     */
    private void localize(WifiFingerprint fingerprint, int floor) {
        int r = findMinimumDistanceRow(fingerprint);
        notifyToAll(new IndoorPosition(mPositions[r], floor, fingerprint.timestamp));
    }

    /**
     * Find the minimum-distanced row (position) from received fingerprint.
     * @param fingerprint Received fingerprint
     * @return index of the minimum distanced row
     */
    private int findMinimumDistanceRow(WifiFingerprint fingerprint) {
        // Reset every distance
        Arrays.fill(mRowDistances, 0);

        // Set filled of possible positions
        HashSet<Integer> solutions = new HashSet<>();

        // For each found AP
        for(SingleAccessPoint ap : fingerprint) {

            // Get AP's possible positions
            HashSet<PositionRss> positions = mBssdPositionRssAssociation.get(ap.bssid);

            // For each position, add it to solutions set and calculate row distance contribute
            for(PositionRss pos : positions) {
                solutions.add(pos.positionIndex);
                mRowDistances[pos.positionIndex] += Math.abs(pos.rssi - ap.level);
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

        return minRow;
    }

    /**
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
     * @return A ready-to-use WiFi fingerprint localizer. Null if an error occurs.
     */
    public static WifiFingerprintLocalizer makeLocalizer(File tsvWellFormattedRSS, int floor) {
        try(BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(tsvWellFormattedRSS)))) {
            // Initialize map of (string,set)
            HashMap<String, HashSet<PositionRss>> sets = new HashMap<>();
            // Initialize positions list
            ArrayList<XYPosition> positions = new ArrayList<>();

            // Initialize sets list. It is useful to have for populating them
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
            String r = null;
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
                        setList.get(i - 2).add(new PositionRss(npos, rssi));
                }

                npos++;
            }

            return new WifiFingerprintLocalizer(sets, (XYPosition[]) positions.toArray(), floor);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        } catch (ArrayIndexOutOfBoundsException e) {
            Log.e("WifiFingLocFactory", "File is not well-formatted");
            return null;
        }
    }

}