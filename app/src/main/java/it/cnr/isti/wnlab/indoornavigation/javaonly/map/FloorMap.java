package it.cnr.isti.wnlab.indoornavigation.javaonly.map;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;

public interface FloorMap {
    boolean isValid(XYPosition p);
    XYPosition getNearestValidPosition(XYPosition p);
}
