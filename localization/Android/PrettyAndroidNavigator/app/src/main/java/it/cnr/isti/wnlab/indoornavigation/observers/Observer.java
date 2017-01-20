package it.cnr.isti.wnlab.indoornavigation.observers;

/**
 * Generic observer.
 * @param <T>
 */
public interface Observer<T> {
    void notify(T data);
}
