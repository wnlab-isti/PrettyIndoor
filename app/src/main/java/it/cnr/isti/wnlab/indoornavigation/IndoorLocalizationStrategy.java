package it.cnr.isti.wnlab.indoornavigation;

import it.cnr.isti.wnlab.indoornavigation.observer.Emitter;

/**
 * A localization strategy is supposed to be an emitter of indoor positions.
 */
public interface IndoorLocalizationStrategy extends Emitter<IndoorPosition> {
    IndoorPosition getCurrentPosition();
}
