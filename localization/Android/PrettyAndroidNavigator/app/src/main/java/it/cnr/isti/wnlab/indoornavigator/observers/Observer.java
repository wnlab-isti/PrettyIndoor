package it.cnr.isti.wnlab.indoornavigator.observers;

/**
 * Generic observer.
 * @param <T>
 */
public interface Observer<T> {
    void notify(T data);
}
