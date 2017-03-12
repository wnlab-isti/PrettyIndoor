package it.cnr.isti.wnlab.indoornavigation.android.wifi;

import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Handler;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigation.emitters.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigation.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.WifiFingerprint;

/**
 * Every mDelay milliseconds scans available access points informations and notifies subscribers.
 */
public class WifiScanner extends AbstractEmitter<WifiFingerprint> implements StartableStoppable {

    public static final int DEFAULT_SCANNING_RATE = 1400;

    private WifiManager mManager;
    private Timer mTimer;
    private long mRate;
    private long mLastTimestamp;
    private Handler mHandler;

    private boolean started = false;

    /**
     * Initialize a WifiScanner with default scanning rate.
     * @param manager
     */
    public WifiScanner(WifiManager manager) {
        commonConstructor(manager);
    }

    /**
     * Initialize a WifiScanner with specified scanning rate.
     * @param manager
     * @param milliseconds
     */
    public WifiScanner(WifiManager manager, long milliseconds) {
        commonConstructor(manager);
        mRate = milliseconds;
    }

    private void commonConstructor(WifiManager manager) {
        mManager = manager;
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
