package it.cnr.isti.wnlab.indoornavigator.framework.util.geomagnetic.mm;

import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.LocationStrategy;
import it.cnr.isti.wnlab.indoornavigator.framework.XYPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.types.MagneticField;

public class MagneticMismatchLocator extends LocationStrategy implements DataObserver<MagneticField> {

    // Positions (rows)
    private XYPosition[] mAllPositions;

    // Magnetic field measured values
    private MagneticField[] mValues;

    // Selected floor. Note it's constant, so it's needed one WifiFingerprintLocator per floor
    private int mFloor;

    // Maximum threshold for distance: if position's row distance is over this, it isn't accepted
    private float mThreshold;

    /**
     * Custom position subtype.
     */
    private class MMFingerprintPosition extends IndoorPosition {

        public MMFingerprintPosition(XYPosition p, int floor, long timestamp) {
            super(p, floor, timestamp);
        }

        public String toString() {
            return "MAG " + super.toString();
        }
    }

    /**
     * @param positions Positions list (rows).
     * @param values Measured magnetic field values.
     * @param floor Floor this localizer refers to.
     */
    private MagneticMismatchLocator(XYPosition[] positions, MagneticField[] values, int floor, float threshold) {
        mAllPositions = positions;
        mValues = values;
        mFloor = floor;
        mThreshold = threshold;
    }

    @Override
    public void notify(MagneticField data) {
        IndoorPosition p = localize(data);
        if(p != null)
            notifyObservers(p);
    }

    /**
     * Calculate minimum euclidean distanced row in matrix and find position.
     * @param mf Magnetic field just measured.
     * @return An average position between the all ones found or null if no one has been found.
     */
    private MMFingerprintPosition localize(MagneticField mf) {
        // All positions which are delta-distanced less than the threshold
        List<XYPosition> positions = new ArrayList<>();

        // Find positions
        for(int i = 0; i < mAllPositions.length; i++) {
            MagneticField v = mValues[i];
            float delta = 0.f;

            // Check x
            float diff = v.x - mf.x;
            delta += diff*diff;

            // Then check y (conditionally)
            if(delta < mThreshold) {
                diff = v.y - mf.y;
                delta += diff*diff;

                // Then check z (conditionally)
                if(delta < mThreshold) {
                    diff = v.z - mf.z;
                    delta += diff*diff;

                    // If x^2+y^2+z^2 < minDelta, update minDelta and position
                    if(delta < mThreshold) {
//                        Log.i("MM", "delta X+Y+Z is " + delta + ", adding position");
                        positions.add(mAllPositions[i]);
                    }
                }
            }
        }

        // Calculate average between found positions
        if(!positions.isEmpty()) {
            float[] newXY = {0.f, 0.f};
            for (XYPosition p : positions) {
                newXY[0] += p.x;
                newXY[1] += p.y;
            }
            newXY[0] /= positions.size();
            newXY[1] /= positions.size();

            // If positions have been found, return an average one
            return new MMFingerprintPosition(new XYPosition(newXY[0], newXY[1]), mFloor, mf.timestamp);
        } else
            return null;
    }

    /**
     * The compatible format for TSV map is:
     *
     * X1\tY1\tMX1\tMY1\tMZ1\n
     * X2\tY2\tMX2\tMY2\tMZ2\n
     * ...
     * XN\tYN\tMXN\tMYN\tMZN
     *
     * NOTE: the table ends without '\n'
     *
     * @return A ready-to-use Magnetic Mismatch locator. Null if an error occurs.
     */
    public static MagneticMismatchLocator makeInstance(File tsvWellFormattedRSSI, int floor, float threshold) {

        try(BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(tsvWellFormattedRSSI)))) {

            // Read and parse all lines of the file
            ArrayList<XYPosition> positions = new ArrayList<>();
            ArrayList<MagneticField> values = new ArrayList<>();
            String r;
            while( (r = br.readLine()) != null ) {
                String[] splitr = r.split("\t");
                positions.add(new XYPosition(Float.parseFloat(splitr[0]), Float.parseFloat(splitr[1])));
                values.add(new MagneticField(Float.parseFloat(splitr[2]), Float.parseFloat(splitr[3]), Float.parseFloat(splitr[4]), -1, -1));
            }

            // Return the ready-to-use MM locator
            XYPosition[] positionArray = positions.toArray(new XYPosition[positions.size()]);
            MagneticField[] valuesArray = values.toArray(new MagneticField[values.size()]);
            return new MagneticMismatchLocator(positionArray, valuesArray, floor, threshold);

        } catch (IOException e) {
            e.printStackTrace();
        } catch (ArrayIndexOutOfBoundsException e) {
            Log.e("WifiFingLocFactory", "File is not well-formatted");
        }

        return null;
    }
}
