package it.cnr.isti.wnlab.indoornavigation.javaonly.types;

import java.io.Serializable;

public class Step implements Serializable {

    public final long timestamp;

    public Step(long timestamp) {
        this.timestamp = timestamp;
    }
}
