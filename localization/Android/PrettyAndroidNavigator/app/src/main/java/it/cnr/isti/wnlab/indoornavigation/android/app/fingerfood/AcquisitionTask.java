package it.cnr.isti.wnlab.indoornavigation.android.app.fingerfood;

import android.os.Handler;
import android.os.Message;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Task for stop acquisition after MILLISECONDS milliseconds.
 */
public class AcquisitionTask implements Runnable {

    private final Handler.Callback mFinishCallback;
    private Timer mTimer;
    private Handler mHandler;

    private int MILLISECONDS = 5000;

    public AcquisitionTask(Handler.Callback finishCallback) {
        mFinishCallback = finishCallback;
        mTimer = new Timer();
        mHandler = new Handler();
    }

    @Override
    public void run() {
        // Start timer
        mTimer.schedule(new TimerTask() {
            public void run() {
                // Notify observers on main thread
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        // Signal finish
                        mFinishCallback.handleMessage(new Message());
                    }
                });
            }
        }, MILLISECONDS);
    }
}