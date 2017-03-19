package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;

public interface PositionPickingStrategy<T extends Particle,P extends XYPosition> {
    P getPosition(Collection<T> particles);
}
