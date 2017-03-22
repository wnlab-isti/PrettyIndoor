package it.cnr.isti.wnlab.indoornavigation.javaonly.utils;

import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigation.javaonly.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;

public abstract class TimedEmitter<T> extends AbstractEmitter<T> implements Observer<T>, StartableStoppable {

    private T mData;
    private Timer mTimer;
    private long mRate;
    private boolean started = false;

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
    public void start() {
        if(!started) {
            mTimer = new Timer();
            mTimer.scheduleAtFixedRate(getTask(), 0, getRate());
            started = true;
        }
    }

    @Override
    public void stop() {
        if(started) {
            mTimer.cancel();
            started = false;
        }
    }

    @Override
    public boolean isStarted() {
        return started;
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
