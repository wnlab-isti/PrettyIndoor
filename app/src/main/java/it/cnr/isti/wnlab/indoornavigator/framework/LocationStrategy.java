package it.cnr.isti.wnlab.indoornavigator.framework;

/**
 * Abstract strategy for 2,5D location (x,y,floor).
 */
public abstract class LocationStrategy {

    private PositionUpdateCallback mUpdater;

    // Position
    protected float x;
    protected float y;
    protected int floor;


    protected LocationStrategy(IndoorPosition startPosition, PositionUpdateCallback updater) {
        x = startPosition.x;
        y = startPosition.y;
        floor = startPosition.floor;
        mUpdater = updater;
    }

    // Position Update methods

    public void setUpdater(PositionUpdateCallback updater) {
        mUpdater = updater;
    }

    public PositionUpdateCallback getUpdater() {
        return mUpdater;
    }

    protected void updatePosition(IndoorPosition position) {
        if(mUpdater != null)
            mUpdater.onPositionUpdate(position);
    }

}
