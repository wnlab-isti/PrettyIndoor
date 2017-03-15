package it.cnr.isti.wnlab.indoornavigation.utils.strategies.fingerprint;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.types.RawData;

public abstract class FingerprintStrategy<T extends RawData> {
    public abstract <P extends XYPosition> Collection<P> localize(T measurement);
}
