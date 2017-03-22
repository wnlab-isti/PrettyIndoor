package it.cnr.isti.wnlab.indoornavigation.android;

import android.os.Handler;

import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Emitter;
import it.cnr.isti.wnlab.indoornavigation.javaonly.utils.TimedEmitter;

public class AndroidTimedEmitter<T> extends TimedEmitter<T> {

    private Handler mHandler;

    public AndroidTimedEmitter(Emitter<T> emitter, long milliseconds) {
        super(emitter, milliseconds);
        mHandler = new Handler();
    }

    @Override
    protected TimerTask getTask() {
        return new TimerTask() {
            @Override
            public void run() {
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        notifyObservers(getData());
                    }
                });
            }
        };
    }
}
