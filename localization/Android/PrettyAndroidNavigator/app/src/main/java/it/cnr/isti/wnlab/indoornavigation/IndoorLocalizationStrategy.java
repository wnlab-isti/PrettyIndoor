package it.cnr.isti.wnlab.indoornavigation;

import it.cnr.isti.wnlab.indoornavigation.observer.Emitter;

public interface IndoorLocalizationStrategy extends Emitter<IndoorPosition> {
    IndoorPosition getCurrentPosition();
}
