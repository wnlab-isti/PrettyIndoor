package it.cnr.isti.wnlab.indoornavigation.observer;

/**
 * Observer for updates (as the well-known design pattern is).
 * @param <T>
 */
public interface Observer<T> {
    void notify(T data);
}
