package it.cnr.isti.wnlab.indoornavigator.framework.callbacks;

/**
 * Callback called when a change in heading is detected.
 */
public interface HeadingChangeCallback {
    void onHeadingChange(float newHeading, long timestamp);
}
