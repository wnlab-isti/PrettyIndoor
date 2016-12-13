package it.cnr.isti.wnlab.indoornavigator.androidutils.wifi;

import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Handler;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import it.cnr.isti.wnlab.indoornavigator.framework.AbstractEmitter;
import it.cnr.isti.wnlab.indoornavigator.framework.DataObserver;
import it.cnr.isti.wnlab.indoornavigator.framework.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigator.framework.types.WifiFingerprint;

/**
 * Every mDelay milliseconds scans available access points informations and notifies subscribers.
 */
public class WifiScanner extends AbstractEmitter<WifiFingerprint> implements StartableStoppable {

    private WifiManager mManager;
    private Timer mTimer;
    private long mRate;
    private DataObserver<WifiFingerprint> mDataReceiver;
    private long mLastTimestamp;
    private Handler mHandler;

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
        // Start timed scan
        mTimer = new Timer();
        mTimer.scheduleAtFixedRate(new TimerTask() {
            public void run() {
                // Adapt previous results and send data to related source
                List<ScanResult> results = mManager.getScanResults();
                final WifiFingerprint data = new WifiFingerprint(results.size(), mLastTimestamp);
                for (ScanResult res : results)
                    data.add(res.BSSID, res.level);
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        mDataReceiver.notify(data);
                    }
                });

                // Start scanning again
                mLastTimestamp = System.currentTimeMillis();
                mManager.startScan();
            }
        }, 0, mRate);
    }

    /**
     * Stop gracefully
     */
    @Override
    public void stop() {
        mTimer.cancel();
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
