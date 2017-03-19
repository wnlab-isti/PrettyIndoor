package it.cnr.isti.wnlab.indoornavigation.javaonly.utils.localization;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.types.RawData;

public abstract class FingerprintStrategy<T extends RawData> {
    public abstract <P extends XYPosition> Collection<P> localize(T measurement);
}
