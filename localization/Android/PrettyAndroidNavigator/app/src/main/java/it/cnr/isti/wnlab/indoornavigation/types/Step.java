package it.cnr.isti.wnlab.indoornavigation.types;

import java.io.Serializable;

/**
 * Step object.
 */
public class Step implements Serializable {

    public final long timestamp;

    public Step(long timestamp) {
        this.timestamp = timestamp;
    }
}
