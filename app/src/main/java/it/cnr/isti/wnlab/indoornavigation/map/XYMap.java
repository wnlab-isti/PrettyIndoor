package it.cnr.isti.wnlab.indoornavigation.map;

/**
 * A 2D map.
 */
public interface XYMap extends IndoorMap {
    boolean isValid(float x, float y);
}
