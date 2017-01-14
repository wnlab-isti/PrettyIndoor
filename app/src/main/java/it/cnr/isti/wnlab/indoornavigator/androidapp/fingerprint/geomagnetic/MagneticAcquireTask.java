package it.cnr.isti.wnlab.indoornavigator.androidapp.fingerprint.geomagnetic;

import android.os.Handler;
import android.os.Message;

import java.io.IOException;
import java.io.Writer;
import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigator.android.sensorhandlers.MagneticFieldHandler;

public class MagneticAcquireTask implements Runnable {

    private Writer mWriter;
    private MagneticFieldHandler mMag;
    private final Handler.Callback mFinishCallback;
    private Handler mHandler;
    private Timer mTimer;

    private int MILLISECONDS = 5000;

    public MagneticAcquireTask(Writer writer, MagneticFieldHandler mag, Handler.Callback finishCallback) {
        mWriter = writer;
        mMag = mag;
        mFinishCallback = finishCallback;
        mTimer = new Timer();
        mHandler = new Handler();
    }

    @Override
    public void run() {
        // Start listening to MF
        mMag.start();

        // Start timer
        mTimer.schedule(new TimerTask() {
            public void run() {
                // Notify observers on main thread
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        // Stop listening MF
                        mMag.stop();

                        // Write '\n' for separating points
                        try {
                            mWriter.write("\n");
                        } catch (IOException e) {
                            e.printStackTrace();
                        }

                        // Signal finish
                        mFinishCallback.handleMessage(new Message());
                    }
                });
            }
        }, MILLISECONDS);
    }
}
