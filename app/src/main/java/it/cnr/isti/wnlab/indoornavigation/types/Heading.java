package it.cnr.isti.wnlab.indoornavigation.types;

import java.io.Serializable;

/**
 * Incapsulates an heading update.
 */
public class Heading implements Serializable {
    public final float heading;
    public final long timestamp;

    public Heading(float heading, long timestamp) {
        this.heading = heading;
        this.timestamp = timestamp;
    }
}
