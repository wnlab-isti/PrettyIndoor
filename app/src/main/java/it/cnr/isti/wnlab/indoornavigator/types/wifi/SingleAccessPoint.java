package it.cnr.isti.wnlab.indoornavigator.types.wifi;

import it.cnr.isti.wnlab.indoornavigator.types.RawData;

/**
 * (bssid,level)
 */
public class SingleAccessPoint implements RawData {
    public final String bssid;
    public final int level;

    public SingleAccessPoint(String bssid , int level) {
        this.bssid = bssid;
        this.level = level;
    }

    @Override
    public String toString() {
        return "(" + bssid + "," + level + ")";
    }
}
