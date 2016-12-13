package it.cnr.isti.wnlab.indoornavigator.framework;

/**
 * Implemented by an object that needs to be started and can be stopped.
 */
public interface StartableStoppable {
    void start();
    void stop();
    boolean isStarted();
}
