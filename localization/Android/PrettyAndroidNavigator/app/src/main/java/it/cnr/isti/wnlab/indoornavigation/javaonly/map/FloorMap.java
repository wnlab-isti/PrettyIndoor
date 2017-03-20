package it.cnr.isti.wnlab.indoornavigation.javaonly.map;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;

public interface FloorMap {
    boolean isValid(XYPosition p);
    boolean isValid(float x, float y);
    XYPosition getNearestValidPosition(XYPosition p);
    XYPosition getNearestValidPosition(float x, float y);
}
