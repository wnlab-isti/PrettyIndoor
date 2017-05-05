package it.cnr.isti.wnlab.indoornavigation.types.fingerprint;

import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.types.environmental.MagneticField;

/**
 * FingerprintMap subtype for Magnetic Fingerprints.
 */
public class MagneticFingerprintMap extends FingerprintMap<XYPosition,MagneticField> {
    @Override
    protected float distanceBetween(MagneticField data1, MagneticField data2) {
        float dx = data1.x - data2.x;
        float dy = data1.y - data2.y;
        float dz = data1.z - data2.z;
        return dx*dx+dy*dy+dz*dz;
    }

    /**
     * Builder class for MagneticFingerprintMap.
     */
    public static class Builder extends FingerprintMapBuilder<MagneticFingerprintMap> {

        public MagneticFingerprintMap build(List<String> lines) {
            // Instantiate fingerprint object
            MagneticFingerprintMap fingerprint = new MagneticFingerprintMap();

            // Parse the text lines
            for (String l : lines) {
                // Split CSV file
                String[] splitLine = l.split(",");

                // Parse coordinate
                XYPosition position = new XYPosition(
                        Float.parseFloat(splitLine[0]),Float.parseFloat(splitLine[1]));

                // Instantiate MF object
                float mx = Float.parseFloat(splitLine[2]);
                float my = Float.parseFloat(splitLine[3]);
                float mz = Float.parseFloat(splitLine[4]);
                MagneticField mf = new MagneticField(mx,my,mz,-1,System.currentTimeMillis());

                // Populate map
                fingerprint.map.put(position, mf);
            }

            // Return fingerprint instance
            return fingerprint;
        }
    }
}
