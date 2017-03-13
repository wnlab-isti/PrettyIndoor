package it.cnr.isti.wnlab.indoornavigation.observer;

/**
 * Generic observer.
 * @param <T>
 */
public interface Observer<T> {
    void notify(T data);
}
