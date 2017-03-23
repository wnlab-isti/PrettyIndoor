package it.cnr.isti.wnlab.indoornavigation.javaonly;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;

public interface IndoorLocalizationStrategy extends Emitter<IndoorPosition> {
    IndoorPosition getCurrentPosition();
}
