package it.cnr.isti.wnlab.indoornavigation.javaonly;

/**
 * Implemented by an object that needs to be started and can be stopped.
 */
public interface StartableStoppable {
    void start();
    void stop();
    boolean isStarted();
}
