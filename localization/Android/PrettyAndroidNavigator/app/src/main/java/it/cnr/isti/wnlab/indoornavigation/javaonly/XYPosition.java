package it.cnr.isti.wnlab.indoornavigation.javaonly;

import java.io.Serializable;

public class XYPosition implements Serializable {
    public final float x;
    public final float y;

    public XYPosition(float x, float y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return x + "," + y;
    }
}
