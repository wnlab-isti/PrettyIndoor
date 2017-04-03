package it.cnr.isti.wnlab.indoornavigation.androidapp.fingerfood;

import com.google.common.collect.HashBasedTable;
import com.google.common.collect.Table;

import java.io.BufferedWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import it.cnr.isti.wnlab.indoornavigation.javaonly.types.fingerprint.WifiFingerprintMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.wifi.SingleAccessPoint;

public class WifiDataMerger extends FingerprintDataMerger {

    // Map of measurements <(x,y),{(String BSSIDi, int RSSIi), ...}>
    private Table<Float, Float, HashMap<String, Integer[]>> measurements;

    // (bssid,rssi) map of the current point
    private HashMap<String, Integer[]> values = null;

    public WifiDataMerger() {
        measurements = HashBasedTable.create();
    }

    @Override
    protected String getLabel() {
        return "W";
    }

    @Override
    protected void setCoordinates(float x, float y) {
        // If the coordinate didn't exist, initialize it in the data structure.
        Map<Float, HashMap<String, Integer[]>> xRow;
        if((xRow = measurements.row(x)) == null || (values = xRow.get(y)) == null) {
            values = new HashMap<>();
            measurements.put(x, y, values);
        }
        // Else, the coordinate already exists and the associated array is assigned to "values".
    }

    /**
     * Expected array format:
     * commaSeparatedValues[0] = "W"
     * cSV[1+2n] = BSSID
     * cSV[2+2n] = RSSI
     * @param commaSeparatedValues Values of a CSV text's line.
     */
    @Override
    protected void insertMeasurement(String[] commaSeparatedValues)
            throws NullPointerException, NumberFormatException {
        if (values != null) {

            for(int i = 1; i < commaSeparatedValues.length; i+=2) {
                // Parse (bssid,rssi) pair
                String bssid = commaSeparatedValues[i];
                int rssi = Integer.parseInt(commaSeparatedValues[i+1]);

                // Insert or update value
                Integer[] prev;
                if((prev = values.get(bssid)) == null)
                    // The value didn't exist, insert it
                    values.put(bssid, new Integer[]{rssi,1});
                else {
                    // The value existed, update it
                    prev[0] += rssi; // Increment value
                    prev[1]++; // Increment cardinality
                }
            }

        } else
            throw new NullPointerException("Current coordinate isn't set.");
    }

    @Override
    protected void merge(BufferedWriter writer) throws IOException {
        // The next cycles are used for iterating forEach(x) -> forEach(y) -> forEach(BSSD)
        Map<Float,Map<Float,HashMap<String, Integer[]>>> rows = measurements.rowMap(); // I'm sorry for this
        // For each row
        for(Map.Entry<Float, Map<Float, HashMap<String,Integer[]>>> row : rows.entrySet()) { // I wish I had forEach(), but I don't
            // For each column
            for(Map.Entry<Float, HashMap<String,Integer[]>> col : row.getValue().entrySet()) { // I'm really sorry for this

                /*
                 * List for sorting. This will be written down on file.
                 */
                ArrayList<SingleAccessPoint> aps = new ArrayList<>();

                // For each BSSID in that coordinate, calculate average value and add a new AP object to the list.
                for(Map.Entry<String, Integer[]> e : col.getValue().entrySet()) { // Ok, here we are ;)
                    /*
                     * Merge measurements (average) and add them to the list.
                     */
                    Integer[] rssi = e.getValue();
                    rssi[0] /= rssi[1];
                    rssi[1] = 1;
                    aps.add(new SingleAccessPoint(e.getKey(),rssi[0]));
                }

                /*
                 * Sort point's BSSID list
                 */
                Collections.sort(aps, AccessPoints.getComparator(WifiFingerprintMap.AP_ORDER_IN_ROW));

                /*
                 * Write down fingerprint row in this format:
                 * x,y,bssid1,rssi1,bssid2,rssi2,bssid3, .. ,rssiN\n
                 */
                if(writer != null) {
                    // Write initial x,y
                    writer.write(row.getKey() + "," + col.getKey());
                    // Write every AP (sorted)
                    for(SingleAccessPoint ap : aps)
                        writer.write("," + ap);
                    // Write newline
                    writer.write("\n");
                }
            }
        }
    }
}
