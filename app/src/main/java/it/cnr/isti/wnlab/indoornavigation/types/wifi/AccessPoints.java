package it.cnr.isti.wnlab.indoornavigation.types.wifi;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.types.RawData;

/**
 * Immutable AccessPoints.
 */
public class AccessPoints implements RawData, Iterable<SingleAccessPoint> {

    // Constants for sorting
    public static final int ORDER_BY_BSSID_ASC = 0;
    public static final int ORDER_BY_BSSID_DESC = 1;
    public static final int ORDER_BY_RSSI_ASC = 2;
    public static final int ORDER_BY_RSSI_DESC = 3;

    // Public timestamp constant field
    public final long timestamp;

    // Private members
    private final SingleAccessPoint[] mApArray;
    private final int mSize;

    public AccessPoints(List<SingleAccessPoint> accessPoints, long timestamp) {
        int size = accessPoints.size();
        mApArray = accessPoints.toArray(new SingleAccessPoint[size]);
        mSize = size;
        this.timestamp = timestamp;
    }

    /**
     * @param p Element's index.
     * @return Requested element or null if it doesn't exists or the index is not valid.
     */
    public SingleAccessPoint get(int p) {
        if(p < 0 || p > mSize-1)
            return null;
        else
            return mApArray[p];
    }

    /**
     * @return Size of the Access Points list.
     */
    public int size() {
        return mSize;
    }

    /**
     * @return A copy of the AP list.
     */
    public SingleAccessPoint[] getArray() {
        return mApArray;
    }

    /**
     * @param order
     * @return A comparator instance for the specified order.
     */
    public static Comparator<SingleAccessPoint> getComparator(int order) {
        // BSSID ASC
        Comparator<SingleAccessPoint> comparator = null;
        if(order == AccessPoints.ORDER_BY_BSSID_ASC)
            comparator = new Comparator<SingleAccessPoint>() {
                @Override
                public int compare(SingleAccessPoint ap1, SingleAccessPoint ap2) {
                    return ap1.bssid.compareTo(ap2.bssid);
                }
            };
        // Other aren't implemented yet.
        else
            throw new UnsupportedOperationException("Not implemented yet");

        // Return comparator instance
        return comparator;
    }

    /**
     * Only BSSID ASC supported at the moment.
     * @param order integer constant corresponding to the specified order.
     */
    public void sort(int order) {
        // Sort array
        Arrays.sort(mApArray, getComparator(order));
    }

    /**
     * Only BSSID ASC supported by now.
     * @param order integer constant corresponding to the specified order.
     * @return true if AP array is sorted by the specified order, false otherwise.
     */
    public boolean isSort(int order) {
        // ORDER BY bssid ASC
        if(order == AccessPoints.ORDER_BY_BSSID_ASC) {
            for (int i = 0; i < mApArray.length - 1; i++)
                if (mApArray[i].bssid.compareTo(mApArray[i + 1].bssid) > 0)
                    return false;
            return true;
        } else
            throw new UnsupportedOperationException("Not implemented yet");

    }

    /**
     * Iteration stuff
     */

    private class WifiIterator implements Iterator<SingleAccessPoint> {

        int i = 0;

        @Override
        public boolean hasNext() {
            return i < mSize-1 && mApArray[i+1] != null;
        }

        @Override
        public SingleAccessPoint next() {
            return mApArray[i++];
        }
    }

    @Override
    public Iterator iterator() {
        return new WifiIterator();
    }

    /**
     * @return W,bssid1,rssi1,bssid2,rssi2, ...
     */
    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder("W");
        for(int i = 0; i < mApArray.length; i++) {
            builder.append(RawData.LOG_SEPARATOR);
            builder.append(mApArray[i]);
        }
        // Removes last LOG_SEPARATOR
        builder.deleteCharAt(builder.length()-1);

        return builder.toString();
    }
}
