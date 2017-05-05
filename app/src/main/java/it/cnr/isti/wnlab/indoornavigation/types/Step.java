package it.cnr.isti.wnlab.indoornavigation.types;

import java.io.Serializable;

public class Step implements Serializable {

    public final long timestamp;

    public Step(long timestamp) {
        this.timestamp = timestamp;
    }
}
