package it.cnr.isti.wnlab.indoornavigation.types;

public class Heading {
    public final float heading;
    public final long timestamp;

    public Heading(float heading, long timestamp) {
        this.heading = heading;
        this.timestamp = timestamp;
    }
}
