package it.cnr.isti.wnlab.indoornavigator.framework.util.intertial.pdr;

import it.cnr.isti.wnlab.indoornavigator.framework.AbstractEmitter;

public abstract class PDR extends AbstractEmitter<PDR.Result> {

    public static class Result {
        public final float dN;
        public final float dE;
        public final float heading;
        public final long timestamp;

        public Result(float dN, float dE, float heading, long timestamp) {
            this.dN = dN;
            this.dE = dE;
            this.heading = heading;
            this.timestamp = timestamp;
        }
    }



}