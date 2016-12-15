package it.cnr.isti.wnlab.indoornavigator.framework.types;

public class Heading {
    public final float heading;
    public final long timestamp;

    public Heading(float heading, long timestamp) {
        this.heading = heading;
        this.timestamp = timestamp;
    }
}
