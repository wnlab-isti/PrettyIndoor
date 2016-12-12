package it.cnr.isti.wnlab.indoornavigator.framework.util.geomagnetic.mm;

import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.LocationStrategy;
import it.cnr.isti.wnlab.indoornavigator.framework.XYPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.types.MagneticField;

public class MagneticMismatchLocator extends LocationStrategy implements DataObserver<MagneticField> {

    // Positions (rows)
    private XYPosition[] mPositions;

    // Magnetic field measured values
    private MagneticField[] mValues;

    // Selected floor. Note it's constant, so it's needed one WifiFingerprintLocator per floor
    private int mFloor;

    // Maximum threshold for distance: if position's row distance is over this, it isn't accepted
    private float mThreshold;

    /**
     * @param positions Positions list (rows).
     * @param values Measured magnetic field values.
     * @param floor Floor this localizer refers to.
     */
    private MagneticMismatchLocator(XYPosition[] positions, MagneticField[] values, int floor, float threshold) {
        mPositions = positions;
        mValues = values;
        mFloor = floor;
        mThreshold = threshold;
    }

    @Override
    public void notify(MagneticField data) {
        notifyObservers(localize(data));
    }

    /**
     * Calculate minimum euclidean distanced row in matrix and find position.
     * @param mf Magnetic field just measured.
     * @return Found IndoorPosition or null.
     */
    private IndoorPosition localize(MagneticField mf) {
        float minDelta = Float.MAX_VALUE;
        XYPosition position = null;

        for(int i = 0; i < mPositions.length; i++) {
            MagneticField v = mValues[i];
            float delta = 0;

            // Check x
            float diff = v.x - mf.x;
            delta += diff*diff;

            // Then check y (conditionally)
            if(delta < minDelta) {
                diff = v.y - mf.y;
                delta += diff*diff;

                // Then check z (conditionally)
                if(delta < minDelta) {
                    diff = v.z - mf.z;
                    delta += diff*diff;

                    // If x^2+y^2+z^2 < minDelta, update minDelta and position
                    if(delta < minDelta) {
                        minDelta = delta;
                        position = mPositions[i];
                    }
                }
            }
        }

        // If a position has been found and its distance is not beyond the threshold
        if(position != null && minDelta < mThreshold)
            return new IndoorPosition(position, mFloor, mf.timestamp);
        else
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
    public static MagneticMismatchLocator makeInstance(File tsvWellFormattedRSSI, int floor, int threshold) {

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

            // Return a ready-to-use MM locator
            return new MagneticMismatchLocator((XYPosition[]) positions.toArray(), (MagneticField[]) values.toArray(), floor, threshold);

        } catch (IOException e) {
            e.printStackTrace();
        } catch (ArrayIndexOutOfBoundsException e) {
            Log.e("WifiFingLocFactory", "File is not well-formatted");
        }

        return null;
    }
}
