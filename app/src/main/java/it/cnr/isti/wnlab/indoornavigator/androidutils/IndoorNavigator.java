package it.cnr.isti.wnlab.indoornavigator.androidutils;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
import it.cnr.isti.wnlab.indoornavigator.framework.PositionUpdateCallback;
import it.cnr.isti.wnlab.indoornavigator.framework.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigator.framework.XYPosition;
import it.cnr.isti.wnlab.indoornavigator.framework.util.geomagnetic.mm.MagneticMismatchLocator;
import it.cnr.isti.wnlab.indoornavigator.framework.util.intertial.pdr.FixedLengthPDR;
import it.cnr.isti.wnlab.indoornavigator.framework.util.strategy.KFUleeStrategy;
import it.cnr.isti.wnlab.indoornavigator.framework.util.wifi.fingerprint.WifiFingerprintLocator;

/**
 * IndoorNavigator object for the user.
 */
public class IndoorNavigator implements StartableStoppable {

    private List<StartableStoppable> mSources;
    private LocationStrategy mStrategy;
    private PositionUpdateCallback mUpdater;

    private static final float DEFAULT_START_X = 0.f;
    private static final float DEFAULT_START_Y = 0.f;
    private static final int DEFAULT_START_FLOOR = 0;

    private static final int WIFI_FINGERPRINT_THRESHOLD = Integer.MAX_VALUE;
    private static final int MM_THRESHOLD = Integer.MAX_VALUE;

    protected IndoorNavigator(StartableStoppable... sources) {
        mSources = Arrays.asList(sources);
    }

    /**
     * Start ALL startable.
     */
    @Override
    public void start() {
        for(StartableStoppable s : mSources)
            s.start();
    }

    /**
     * Stop ALL stoppable.
     */
    @Override
    public void stop() {
        for(StartableStoppable s : mSources)
            s.stop();
    }

    // NOTE: sources are immutable.

    // Strategy getter and (private) setter
    public LocationStrategy getStrategy() { return mStrategy; }
    private void setStrategy(LocationStrategy strategy) { mStrategy = strategy; }

    // XYPosition Updater getter and setter
    public void setPositionUpdater(PositionUpdateCallback updater) {
        mUpdater = updater;
    }
    public PositionUpdateCallback getPositionUpdater() { return mUpdater; }

    /******************************************************************************************/

    /**
     * Factory class for IndoorNavigator.
     */
    public static class Builder {

        private List<StartableStoppable> mSources;
        private LocationStrategy mStrategy;
        private PositionUpdateCallback mUpdater;
        private File mWifiFingerprintMap;
        private File mMagneticFingerprintMap;

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
        public void setPositionUpdater(PositionUpdateCallback updater) {
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
                for(StartableStoppable s : mSources) {
                    Class<?> type = s.getClass();
                    if (type == AccelerometerHandler.class)
                        acc = (AccelerometerHandler) s;
                    else if (type == GyroscopeHandler.class)
                        gyro = (GyroscopeHandler) s;
                    else if(type == MagneticFieldHandler.class)
                        mag = (MagneticFieldHandler) s;
                    else if(type == WifiScanner.class)
                        wifi = (WifiScanner) s;
                }

                // Choose strategy and link everything
                List<StartableStoppable> sources = new ArrayList<>();
                if(acc != null && mag != null && gyro != null) {
                    // Pseudo-You Li strategy
                    FixedLengthPDR pdr = new FixedLengthPDR(0.f);
                    WifiFingerprintLocator wifiLoc = WifiFingerprintLocator.makeInstance(
                            mWifiFingerprintMap,0,WIFI_FINGERPRINT_THRESHOLD);
                    MagneticMismatchLocator mm = MagneticMismatchLocator.makeInstance(
                            mMagneticFingerprintMap,0,MM_THRESHOLD);
                    KFUleeStrategy strategy = new KFUleeStrategy(
                            new IndoorPosition(
                                    DEFAULT_START_X, DEFAULT_START_Y,
                                    DEFAULT_START_FLOOR,
                                    System.currentTimeMillis()),
                            pdr,
                            wifiLoc,
                            mm,
                            mUpdater);

                    // Heading from sensor fusion
                    Compass heading = new LawitzkiCompass(pdr, acc, gyro, mag, 30);
                    sources.add(heading);

                    // Step detection
                    StepDetector sd = new FasterStepDetector(pdr, acc);
                    sources.add(sd);

                    // Set strategy as the chosen one
                    mStrategy = strategy;
                }

                // Initialize IndoorNavigator instance
                IndoorNavigator nav = new IndoorNavigator((StartableStoppable[]) sources.toArray());

                // Set strategy
                nav.setStrategy(mStrategy);

                // Set position updater
                nav.setPositionUpdater(mUpdater);

                // Return built instance
                return nav;
            } else
                return null;
        }

    }
}
