package it.cnr.isti.wnlab.indoornavigation.androidapp;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;

public class MyISTIFloorMap implements FloorMap {

    @Override
    public boolean isValid(XYPosition p) {
        return true;
    }

    @Override
    public boolean isValid(float x, float y) {
        return true;
    }

    @Override
    public XYPosition getNearestValidPosition(XYPosition p) {
        return getNearestValidPosition(p.x,p.y);
    }

    @Override
    public XYPosition getNearestValidPosition(float x, float y) {
        return null;
    }
}
