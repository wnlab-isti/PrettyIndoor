package it.cnr.isti.wnlab.indoornavigation.utils;

import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;

public abstract class TimedEmitter<T> extends AbstractEmitter<T> implements Observer<T> {

    private T mData;
    private Timer mTimer;
    private long mRate;

    public TimedEmitter(Emitter<T> emitter, long milliseconds) {
        // Register to emitter
        emitter.register(this);
        // Delay
        mRate = milliseconds;
    }

    @Override
    public void notify(T data) {
        mData = data;
    }


    @Override
    protected void start() {
        mTimer = new Timer();
        mTimer.scheduleAtFixedRate(getTask(), 0, getRate());
    }

    @Override
    protected void stop() {
        mTimer.cancel();
    }

    /**
     * @return The task to execute every mRate milliseconds.
     */
    protected abstract TimerTask getTask();

    /**
     * @return Last received data.
     */
    protected T getData() {
        return mData;
    }

    /*
     * Rate getter/setter
     */

    public long getRate() {
        return mRate;
    }

    public void setRate(int mRate) {
        this.mRate = mRate;
    }
}
