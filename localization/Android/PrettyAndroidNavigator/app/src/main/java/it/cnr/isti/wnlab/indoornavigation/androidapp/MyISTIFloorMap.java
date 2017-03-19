package it.cnr.isti.wnlab.indoornavigation.androidapp;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;

public class MyISTIFloorMap implements FloorMap {

    @Override
    public boolean isValid(XYPosition p) {
        return false;
    }

    @Override
    public XYPosition getNearestValidPosition(XYPosition p) {
        return null;
    }

}
