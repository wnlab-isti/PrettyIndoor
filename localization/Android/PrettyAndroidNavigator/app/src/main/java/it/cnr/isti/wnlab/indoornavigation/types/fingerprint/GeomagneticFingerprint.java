package it.cnr.isti.wnlab.indoornavigation.types.fingerprint;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.types.environmental.MagneticField;

public class GeomagneticFingerprint extends Fingerprint<XYPosition,MagneticField> {
    @Override
    protected float distanceBetween(MagneticField data1, MagneticField data2) {
        float dx = data1.x - data2.x;
        float dy = data1.y - data2.y;
        float dz = data1.z - data2.z;
        return dx*dx-dy*dy-dz*dz;
    }
}
