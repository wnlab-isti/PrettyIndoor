package it.cnr.isti.wnlab.indoornavigator.framework;

/**
 * Called in order to signal a new position.
 */
public interface PositionUpdateCallback {
    void onPositionUpdate(IndoorPosition newPosition);
}
