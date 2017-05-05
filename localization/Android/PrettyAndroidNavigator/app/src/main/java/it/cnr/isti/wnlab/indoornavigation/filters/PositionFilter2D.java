package it.cnr.isti.wnlab.indoornavigation.filters;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;

/**
 * StateEstimationFilter that operates with x,y coordinates.
 */
public interface PositionFilter2D extends StateEstimationFilter {
    /**
     * @return Object representing last found position.
     */
    XYPosition get2DPosition();
}
