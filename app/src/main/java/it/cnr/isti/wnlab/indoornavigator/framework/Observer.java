package it.cnr.isti.wnlab.indoornavigator.framework;

/**
 * Generic observer.
 * @param <T>
 */
public interface Observer<T> {
    void notify(T data);
}
