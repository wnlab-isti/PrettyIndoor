package it.cnr.isti.wnlab.indoornavigation.javaonly;

public class XYPosition {
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
