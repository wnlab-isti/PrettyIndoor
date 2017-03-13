package it.cnr.isti.wnlab.indoornavigation.filters.kalmanfilter.adapters;

import java.util.Arrays;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.LocationStrategy;
import it.cnr.isti.wnlab.indoornavigation.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.filters.kalmanfilter.IndoorKalmanFilter;

public class LocationStrategyUpdater extends KalmanFilterUpdater {

    private List<LocationStrategy> mStrategies;
    private XYPosition[] mPositions;
    private int mPositionCount;

    // Customized Kalman Filter
    private IndoorKalmanFilter filter;

    // Indexing of strategies/positions
    private static int index = 0;

    public LocationStrategyUpdater(
            IndoorKalmanFilter filter,
            LocationStrategy... strategies) {
        super(filter);
        this.filter = filter;

        // Create list of strategies
        mStrategies = Arrays.asList(strategies);
        // Iterate on strategies for adding observers
        for(LocationStrategy s : mStrategies) {
            s.register(new Observer<IndoorPosition>() {
                private final int strategyIndex = index;
                @Override
                public void notify(IndoorPosition position) {
                    savePosition(strategyIndex, position);
                }
            });
            index++;
        }
        // Initialize positions arrays
        mPositions = new XYPosition[strategies.length];
        clearPositions();
    }

    /**
     * Remove all saved positions.
     */
    public void clearPositions() {
        for(int i = 0; i < mPositions.length; i++)
            mPositions[i] = null;
        mPositionCount = 0;
    }

    /**
     * @param i The caller strategy index in list.
     * @param position Found position.
     */
    private void savePosition(int i, XYPosition position) {
        // Set the received position
        if(mPositions[i] == null)
            mPositionCount++;

        // Set received position
        mPositions[i] = position;

        // If every strategy has given a position, do update
        if(mPositionCount == mStrategies.size()) {
            // Do update
            doKFUpdate();
            // Reset positions
            clearPositions();
        }
    }

    /**
     * Calculate dX,dY for every position and do update.
     */
    private void doKFUpdate() {
        // Get current filter's position
        XYPosition currentPosition = filter.get2DPosition();
        float[] z = {Float.MAX_VALUE, Float.MAX_VALUE, 0.f, 0.f};

        // Find nearest position to current
        for(XYPosition p : mPositions) {
            float dx = currentPosition.x - p.x;
            float dy = currentPosition.y - p.y;
            // If distance is minor, replace nearest position
            if( ((dx*dx)+(dy*dy)) < ((z[0]*z[0])+(z[1]*z[1])) ) {
                z[0] = dx;
                z[1] = dy;
            }
        }

        // Update filter with nearest position
        filter.update(z);
    }
}
