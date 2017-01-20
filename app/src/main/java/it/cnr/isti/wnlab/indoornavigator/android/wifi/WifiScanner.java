package it.cnr.isti.wnlab.indoornavigator.android.wifi;

import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Handler;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigator.emitters.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigator.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigator.types.wifi.WifiFingerprint;

/**
 * Every mDelay milliseconds scans available access points informations and notifies subscribers.
 */
public class WifiScanner extends AbstractEmitter<WifiFingerprint> implements StartableStoppable {

    private WifiManager mManager;
    private Timer mTimer;
    private long mRate;
    private long mLastTimestamp;
    private Handler mHandler;

    private boolean started = false;

    public WifiScanner(WifiManager manager, long milliseconds) {
        mManager = manager;
        mRate = milliseconds;
        mLastTimestamp = System.currentTimeMillis();
        mHandler = new Handler();
    }

    /**
     * Start scanning
     */
    @Override
    public void start() {
        if(!started) {
            // Start timed scan
            mTimer = new Timer();
            mTimer.scheduleAtFixedRate(new TimerTask() {
                public void run() {
                    // Adapt previous results and send data to related source
                    List<ScanResult> results = mManager.getScanResults();
                    final WifiFingerprint data = new WifiFingerprint(results.size(), mLastTimestamp);
                    for (ScanResult res : results)
                        data.add(res.BSSID, res.level);
                    // Notify observers on main thread
                    mHandler.post(new Runnable() {
                        @Override
                        public void run() {
                            notifyNewFingerprint(data);
                        }
                    });

                    // Start scanning again
                    mLastTimestamp = System.currentTimeMillis();
                    mManager.startScan();
                }
            }, 0, mRate);

            started = true;
        }
    }

    private void notifyNewFingerprint(WifiFingerprint f) {
        notifyObservers(f);
    }

    /**
     * Stop gracefully
     */
    @Override
    public void stop() {
        if(started) {
            mTimer.cancel();
            started = true;
        }
    }

    @Override
    public boolean isStarted() {
        return started;
    }

    /**
     * Rate getter and setter
     */

    public long getRate() {
        return mRate;
    }

    public void setRate(long newRate) {
        mRate = newRate;
    }

}
