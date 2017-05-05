package it.cnr.isti.wnlab.indoornavigation.map;

public interface XYMap extends IndoorMap {
    boolean isValid(float x, float y);
}
