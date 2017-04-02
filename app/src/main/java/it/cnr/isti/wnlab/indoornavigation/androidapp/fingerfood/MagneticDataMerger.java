package it.cnr.isti.wnlab.indoornavigation.androidapp.fingerprint;

import android.util.Log;

import com.google.common.collect.HashBasedTable;
import com.google.common.collect.Table;

import java.io.BufferedWriter;
import java.io.IOException;
import java.util.Map;

public class MagneticDataMerger extends FingerprintDataMerger {

    // Map of measurements <(x,y),(mx,my,mz)>
    private Table<Float,Float,Float[]> measurements;

    // Current array of values
    private Float[] values = null;

    public MagneticDataMerger() {
        measurements = HashBasedTable.create();
    }

    @Override
    protected String getLabel() {
        return "M";
    }

    @Override
    protected void setCoordinates(float x, float y) {
        // If the coordinate didn't exist, initialize it in the data structure.
        Map<Float, Float[]> xRow;
        if((xRow = measurements.row(x)) == null || (values = xRow.get(y)) == null) {
            values = new Float[4];
            values[0] = 0.f;
            values[1] = 0.f;
            values[2] = 0.f;
            values[3] = 0.f;
            measurements.put(x, y, values);
        }
        // Else, the coordinate exists and the associated array is assigned to "values".
    }

    @Override
    protected void insertMeasurement(String[] commaSeparatedValues) {
        if (values != null) {
            // Parse values
            float mx = Float.parseFloat(commaSeparatedValues[2]);
            float my = Float.parseFloat(commaSeparatedValues[3]);
            float mz = Float.parseFloat(commaSeparatedValues[4]);

            Log.d("MAGFP", "Values != null, inserting " + mx + "," + my + "," + mz);

            // Update the coordinate measurements
            values[0] += mx;
            values[1] += my;
            values[2] += mz;
            values[3]++;
        } else
            throw new NullPointerException("Current coordinate isn't set.");
    }

    @Override
    protected void merge(BufferedWriter writer) throws IOException {

        Log.d("MAGFP", "Merging now");

        Map<Float, Map<Float,Float[]>> rows = measurements.rowMap();
        // For each row
        for(Map.Entry<Float, Map<Float, Float[]>> row : rows.entrySet()) {
            // For each column
            for(Map.Entry<Float, Float[]> col : row.getValue().entrySet()) {
                // Do the average and change measurements cardinality to 1
                Float[] values = col.getValue();
                float n = values[3];
                values[0] /= n;
                values[1] /= n;
                values[2] /= n;
                values[3] = 1.f;

                // Log
                Log.d("MAGFP", row.getKey() + "," + col.getKey() + "," +
                        values[0] + "," + values[1] + "," + values[2]);

                // Write on file
                writer.write(
                        row.getKey() + "," + col.getKey() + "," + // (x,y)
                        values[0] + "," + values[1] + "," + values[2] + "\n" // (mx,my,mz)
                );
            }
        }
    }
}
