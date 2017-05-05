package it.cnr.isti.wnlab.indoornavigation.android.handlers;

import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Handler;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigation.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.AccessPoints;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.SingleAccessPoint;

/**
 * Every mDelay milliseconds scans available access points informations and notifies subscribers.
 */
public class WifiScanner extends DataEmitter<AccessPoints> {

    private WifiManager mManager;
    private Timer mTimer;
    private long mRate;
    private long mLastTimestamp;
    private Handler mHandler;

    private boolean active;

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
        active = false;
        mManager = manager;
        mLastTimestamp = System.currentTimeMillis();
        mHandler = new Handler();
    }

    /**
     * Start scanning
     */
    @Override
    public void startEmission() {
        if(!active) {
            // Start timed scan
            mTimer = new Timer();
            mTimer.scheduleAtFixedRate(new TimerTask() {
                public void run() {
                    // Adapt previous results and send data to related source
                    List<ScanResult> results = mManager.getScanResults();
                    List<SingleAccessPoint> aps = new ArrayList<>();
                    for (ScanResult res : results)
                        aps.add(new SingleAccessPoint(res.BSSID, res.level));

                    // Create the AccessPoints instance to return
                    final AccessPoints data = new AccessPoints(aps, mLastTimestamp);

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

            active = true;
        }
    }

    private void notifyNewFingerprint(AccessPoints f) {
        notifyObservers(f);
    }

    /**
     * Stop gracefully
     */
    @Override
    public void stopEmission() {
        if (active) {
            mTimer.cancel();
            active = false;
        }
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
