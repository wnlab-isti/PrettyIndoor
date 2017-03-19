package it.cnr.isti.wnlab.indoornavigation.javaonly.observer;

/**
 * Generic observer.
 * @param <T>
 */
public interface Observer<T> {
    void notify(T data);
}
