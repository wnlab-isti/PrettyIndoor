package it.cnr.isti.wnlab.indoornavigation;

/**
 * Created by m on 10/12/16.
 */

public class XYPosition {
    public final float x;
    public final float y;

    public XYPosition(float x, float y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "(" + x + "," + y + ")";
    }
}
