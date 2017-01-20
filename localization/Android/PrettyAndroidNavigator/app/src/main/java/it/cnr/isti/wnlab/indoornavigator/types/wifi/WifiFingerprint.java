package it.cnr.isti.wnlab.indoornavigator.types.wifi;

import java.util.Iterator;

import it.cnr.isti.wnlab.indoornavigator.types.RawData;

/**
 * Immutable WifiFingerprint.
 */
public class WifiFingerprint implements RawData, Iterable<SingleAccessPoint> {

    private final SingleAccessPoint[] mApArray;
    public final long timestamp;
    private int i = 0;
    private final int mSize;

    public WifiFingerprint(int size, long timestamp) {
        mApArray = new SingleAccessPoint[size];
        mSize = size;
        this.timestamp = timestamp;
    }

    /**
     * @param bssid
     * @param level
     * @return true if the element was successfully insterted, false otherwise.
     */
    public boolean add(String bssid, int level) {
        if(i < mSize) {
            mApArray[i] = new SingleAccessPoint(bssid, level);
            i++;
            return true;
        } else
            return false;
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
     * @return W (bssid1,rssi1) (bssid2,rssi2) ...
     */
    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder("W ");
        for(int i = 0; i < mApArray.length; i++) {
            builder.append(mApArray[i] + " ");
        }
        builder.append('\n');
        return builder.toString();
    }
}
