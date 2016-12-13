package it.cnr.isti.wnlab.indoornavigator.androidutils;

import android.util.Log;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigator.android.Logger;
import it.cnr.isti.wnlab.indoornavigator.android.PositionLogger;
import it.cnr.isti.wnlab.indoornavigator.androidutils.compass.Compass;
import it.cnr.isti.wnlab.indoornavigator.androidutils.compass.LawitzkiCompass;
import it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigator.androidutils.sensorhandlers.MagneticFieldHandler;
import it.cnr.isti.wnlab.indoornavigator.androidutils.stepdetection.FasterStepDetector;
import it.cnr.isti.wnlab.indoornavigator.androidutils.stepdetection.StepDetector;
import it.cnr.isti.wnlab.indoornavigator.androidutils.wifi.WifiScanner;
import it.cnr.isti.wnlab.indoornavigator.framework.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.LocationStrategy;
import it.cnr.isti.wnlab.indoornavigator.framework.Observer;
import it.cnr.isti.wnlab.indoornavigator.framework.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigator.framework.util.geomagnetic.mm.MagneticMismatchLocator;
import it.cnr.isti.wnlab.indoornavigator.framework.util.intertial.pdr.FixedLengthPDR;
import it.cnr.isti.wnlab.indoornavigator.framework.util.strategy.KFUleeStrategy;
import it.cnr.isti.wnlab.indoornavigator.framework.util.wifi.fingerprint.WifiFingerprintLocator;

/**
 * IndoorNavigator object for the user.
 */
public class IndoorNavigator implements StartableStoppable, Observer<IndoorPosition> {

    private List<StartableStoppable> mSources;
    private LocationStrategy mStrategy;
    private Observer<IndoorPosition> mUpdater;

    private static final float DEFAULT_START_X = 0.f;
    private static final float DEFAULT_START_Y = 0.f;
    private static final int DEFAULT_START_FLOOR = 0;

    private static final int WIFI_FINGERPRINT_THRESHOLD = Integer.MAX_VALUE;
    private static final float MM_THRESHOLD = 10.f;

    private boolean started = false;

    protected IndoorNavigator(List<StartableStoppable> sources) {
        mSources = sources;
    }

    /**
     * Start ALL startable.
     */
    @Override
    public void start() {
        if(!started) {
            for (StartableStoppable s : mSources)
                s.start();
            started = true;
        }
    }

    /**
     * Stop ALL stoppable.
     */
    @Override
    public void stop() {
        if(started) {
            for (StartableStoppable s : mSources)
                s.stop();
            started = false;
        }
    }

    @Override
    public boolean isStarted() {
        return started;
    }

    // NOTE: sources are immutable.

    // Strategy getter and (private) setter
    public LocationStrategy getStrategy() { return mStrategy; }
    private void setStrategy(LocationStrategy strategy) {
        // Remove an older strategy, if it exists
        if(mStrategy != null)
            mStrategy.unregister(this);
        // Set new strategy and register the navigator to its updates
        mStrategy = strategy;
        mStrategy.register(this);
    }

    // XYPosition Updater getter and setter
    public void setPositionUpdater(Observer<IndoorPosition> updater) {
        mUpdater = updater;
    }
    public Observer<IndoorPosition> getPositionUpdater() { return mUpdater; }

    @Override
    public void notify(IndoorPosition strategyPosition) {
        mUpdater.notify(strategyPosition);
    }

    /******************************************************************************************/

    /**
     * Factory class for IndoorNavigator.
     */
    public static class Builder {

        private List<StartableStoppable> mSources;
        private LocationStrategy mStrategy;
        private Observer<IndoorPosition> mUpdater;
        private File mWifiFingerprintMap;
        private File mMagneticFingerprintMap;

        private Logger mDataLogger;
        private PositionLogger mWifiFingerprintPositionLogger;
        private PositionLogger mMMPositionLogger;

        /**
         * Set data sources and pre-filter fusion techniques like Step Detection and Heading.
         * @param sources
         */
        public void setSources(List<StartableStoppable> sources) {
            mSources = sources;
        }

        /**
         * Set a new position update callback.
         * @param updater
         */
        public void setPositionObserver(Observer<IndoorPosition> updater) {
            mUpdater = updater;
        }

        /**
         * Set a well-formatted TSV for Wifi fingerprints.
         * @param wifiFingerprintMap
         */
        public void setWifiFingerprintMap(File wifiFingerprintMap) {
            mWifiFingerprintMap = wifiFingerprintMap;
        }

        /**
         * Set a well-formatted TSV for magnetic field fingerprints.
         * @param magneticFingerprintMap
         */
        public void setMagneticFingerprintMap(File magneticFingerprintMap) {
            mMagneticFingerprintMap = magneticFingerprintMap;
        }

        /**
         * Set logger for data.
         * @param logger
         */
        public void setDataLogger(Logger logger) {
            mDataLogger = logger;
        }

        /**
         * Set logger for wifi fingerprint positioning.
         * @param logger
         */
        public void setWifiFingerprintLogger(PositionLogger logger) {
            mWifiFingerprintPositionLogger = logger;
        }

        /**
         * Set logger for mm fingerprint positioning.
         * @param logger
         */
        public void setMMFingerprintLogger(PositionLogger logger) {
            mMMPositionLogger = logger;
        }

        /**
         * @return A ready-to-use startable IndoorNavigator or null if the builder is not properly configured.
         */
        public IndoorNavigator create() {
            // Check if there's everything
            if(mSources != null && mUpdater != null) {

                // Scan available sources
                AccelerometerHandler acc = null;
                GyroscopeHandler gyro = null;
                MagneticFieldHandler mag = null;
                WifiScanner wifi = null;
                for (StartableStoppable s : mSources) {
                    Class<?> type = s.getClass();
                    Log.d("ITERATION", "Current class is " + type.getName());
                    if (type == AccelerometerHandler.class)
                        acc = (AccelerometerHandler) s;
                    else if (type == GyroscopeHandler.class)
                        gyro = (GyroscopeHandler) s;
                    else if (type == MagneticFieldHandler.class)
                        mag = (MagneticFieldHandler) s;
                    else if (type == WifiScanner.class)
                        wifi = (WifiScanner) s;
                }
                Log.d("SCANRESULT", "sources empty? " + mSources.isEmpty() + " | " +
                        "acc" + acc + ", gyro " + gyro + ", mag " + mag + ", wifi " + wifi);

                // Wifi Fingerprint
                WifiFingerprintLocator wifiLoc = null;
                if (wifi != null && mWifiFingerprintMap != null) {
                    // Build locator instance
                    wifiLoc = WifiFingerprintLocator.makeInstance(
                            mWifiFingerprintMap, 0, WIFI_FINGERPRINT_THRESHOLD);
                    // Register locator to wifi emitter
                    wifi.register(wifiLoc);
                    // Register logger to wifi location
                    if(mWifiFingerprintPositionLogger != null)
                        wifiLoc.register(mWifiFingerprintPositionLogger);
                }

                // Geomagnetic fingerprint
                MagneticMismatchLocator mm = null;
                if (mag != null && mMagneticFingerprintMap != null) {
                    // Build locator instance
                    mm = MagneticMismatchLocator.makeInstance(
                            mMagneticFingerprintMap, 0, MM_THRESHOLD);
                    // Register locator to magnetic emitter
                    mag.register(mm);
                    // Register logger to mm location
                    if(mMMPositionLogger != null)
                        mm.register(mMMPositionLogger);
                }

                // Choose strategy and link everything
                if (acc != null && mag != null && gyro != null) {
                    Log.d("INBUILDER","PDR recognised");
                    // Pseudo-You Li strategy
                    FixedLengthPDR pdr = new FixedLengthPDR(0.f);
                    // Heading
                    Compass heading = new LawitzkiCompass(pdr, acc, gyro, mag, 30);
                    mSources.add(heading);
                    // Step detection
                    StepDetector sd = new FasterStepDetector(pdr, acc);
                    mSources.add(sd);

                    // Pseudo-You Li strategy with KF
                    mStrategy = new KFUleeStrategy(
                            new IndoorPosition(
                                    DEFAULT_START_X, DEFAULT_START_Y,
                                    DEFAULT_START_FLOOR,
                                    System.currentTimeMillis()),
                            pdr,
                            wifiLoc,
                            mm);

                } else if(wifi != null) {
                    Log.d("INBUILDER","Wifi-only recognised");
                    // If PDR isn't available but WiFi is, go Wifi fingerprint-only
                    mStrategy = wifiLoc;
                } else if(mag != null) {
                    Log.d("INBUILDER","MM-only recognised");
                    // If PDR and WiFi aren't available, go MM-only
                    mStrategy = mm;
                }

                // Initialize IndoorNavigator instance
                IndoorNavigator nav = new IndoorNavigator(mSources);

                // Set strategy
                nav.setStrategy(mStrategy);

                // Set position updater
                nav.setPositionUpdater(mUpdater);

                // Return built instance
                return nav;
            }

            return null;
        }
    }
}
