package it.cnr.isti.wnlab.indoornavigation.javaonly.filters;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;

/**
 * Filter that operates with x,y coordinates.
 */
public interface PositionFilter2D extends Filter {
    /**
     * @return Object representing last found position.
     */
    XYPosition get2DPosition();
}
