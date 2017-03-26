package it.cnr.isti.wnlab.indoornavigation.javaonly.utils;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;

public class GeometryUtils {

    private GeometryUtils() {}

    public static boolean isPointInCircle(float x, float y, float cx, float cy, float radius) {
        float dx = cx-x;
        float dy = cy-y;
        float d = (float) Math.sqrt(dx*dx+dy*dy);
        return d <= radius;
    }

    public static boolean isPointInCircle(XYPosition point, XYPosition center, float radius) {
        return isPointInCircle(point.x,point.y,center.x,center.y,radius);
    }
}
