package it.cnr.isti.wnlab.indoornavigation.androidapp.fingerprint;

import android.os.Handler;
import android.os.Message;

import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigation.emitters.Emitter;

public class AcquisitionTask implements Runnable {

    private final Handler.Callback mFinishCallback;
    private Timer mTimer;
    private Handler mHandler;
    private Set<Emitter> mEmitters;

    private int MILLISECONDS = 5000;

    public AcquisitionTask(Set<Emitter> emitters, Handler.Callback finishCallback) {
        mFinishCallback = finishCallback;
        mTimer = new Timer();
        mEmitters = emitters;
        mHandler = new Handler();
    }

    @Override
    public void run() {
        // Start timer
        mTimer.schedule(new TimerTask() {
            public void run() {
                // Start emitters for listening
                startEmitters();

                // Notify observers on main thread
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        // Stop data emitters
                        stopEmitters();

                        // Signal finish
                        mFinishCallback.handleMessage(new Message());
                    }
                });
            }
        }, MILLISECONDS);
    }

    private void startEmitters() {
        for(Emitter e : mEmitters)
            e.start();
    }

    private void stopEmitters() {
        for(Emitter e : mEmitters)
            e.stop();
    }
}