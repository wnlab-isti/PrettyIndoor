package it.cnr.isti.wnlab.indoornavigation.android;

import android.content.Context;
import android.util.Log;
import android.widget.Toast;

import java.io.File;
import java.io.Writer;
import java.lang.ref.WeakReference;
import java.util.List;

import it.cnr.isti.wnlab.indoornavigation.IndoorNavigator;
import it.cnr.isti.wnlab.indoornavigation.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.LocationStrategy;
import it.cnr.isti.wnlab.indoornavigation.R;
import it.cnr.isti.wnlab.indoornavigation.android.compass.Compass;
import it.cnr.isti.wnlab.indoornavigation.android.compass.RelativeCompass;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.AccelerometerHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.GyroscopeHandler;
import it.cnr.isti.wnlab.indoornavigation.android.handlers.MagneticFieldHandler;
import it.cnr.isti.wnlab.indoornavigation.android.stepdetection.FasterStepDetector;
import it.cnr.isti.wnlab.indoornavigation.android.stepdetection.StepDetector;
import it.cnr.isti.wnlab.indoornavigation.android.wifi.WifiScanner;
import it.cnr.isti.wnlab.indoornavigation.androidapp.Logger;
import it.cnr.isti.wnlab.indoornavigation.androidapp.PositionLogger;
import it.cnr.isti.wnlab.indoornavigation.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.StartableStoppable;
import it.cnr.isti.wnlab.indoornavigation.types.Heading;
import it.cnr.isti.wnlab.indoornavigation.types.environmental.MagneticField;
import it.cnr.isti.wnlab.indoornavigation.types.inertial.Acceleration;
import it.cnr.isti.wnlab.indoornavigation.types.inertial.AngularSpeed;
import it.cnr.isti.wnlab.indoornavigation.types.wifi.WifiFingerprint;
import it.cnr.isti.wnlab.indoornavigation.utils.strategy.geomagnetic.KnnMagneticMismatch;
import it.cnr.isti.wnlab.indoornavigation.utils.intertial.pdr.FixedLengthPDR;
import it.cnr.isti.wnlab.indoornavigation.utils.strategy.fusion.SimpleKalmanFilterStrategy;
import it.cnr.isti.wnlab.indoornavigation.utils.strategy.wifi.KnnWifiFingerprint;

/**
 * AndroidIndoorNavigator object for the user.
 */
public class AndroidIndoorNavigator implements IndoorNavigator {

    private List<StartableStoppable> mSources;
    private LocationStrategy mStrategy;
    private Observer<IndoorPosition> mUpdater;

    private boolean started = false;

    protected AndroidIndoorNavigator(List<StartableStoppable> sources) {
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

    // Strategy setter
    private void setStrategy(LocationStrategy strategy) {
        if(strategy != null) {
            // Remove an older strategy, if it exists
            if (mUpdater != null) mStrategy.unregister(mUpdater);
            // Set new strategy and register the navigator to its updates
            mStrategy = strategy;
            if (mUpdater != null) mStrategy.register(mUpdater);
        }
    }

    // XYPosition Updater getter and setter
    @Override
    public void setPositionUpdater(Observer<IndoorPosition> updater) {
        if(updater != null) {
            if (mStrategy != null) mStrategy.unregister(mUpdater);
            mUpdater = updater;
            if (mStrategy != null) mStrategy.register(updater);
        }
    }
    @Override
    public Observer<IndoorPosition> getPositionUpdater() { return mUpdater; }

    /******************************************************************************************/

    /**
     * Factory class for AndroidIndoorNavigator.
     */
    public static class Builder {

        // AndroidIndoorNavigator that will be made by this builder
        private AndroidIndoorNavigator nav;

        // Sources which send data to the whole thing
        private List<StartableStoppable> mSources;

        // Preferred strategy (if any)
        private LocationStrategy mStrategy;

        // Observer for new positions
        private Observer<IndoorPosition> mUpdater;

        // Wifi fingerprint map
        private File mWifiFingerprintMap;
        // Geomagnetic fingerprint map
        private File mMagneticFingerprintMap;

        // Logging
        private Writer mLogWriter;
        private PositionLogger mWifiFingerprintPositionLogger;
        private PositionLogger mMMPositionLogger;

        // Position where user starts
        private IndoorPosition mInitialPosition;

        // Members for output communication
        private boolean builderIsSocial;
        private WeakReference<Context> context;

        // Threshold constants
        private static final int WIFI_FINGERPRINT_THRESHOLD = Integer.MAX_VALUE;
        private static final int MM_KNN = 10;
        private static final float MM_THRESHOLD = 10.f;

        /**
         * A silent AndroidIndoorNavigator factory object.
         */
        public Builder() {
            builderIsSocial = false;
        }

        /**
         * A communicative AndroidIndoorNavigator factory object (mainly through Toast notifications).
         * @param context The context to communicate to.
         */
        public Builder(Context context) {
            builderIsSocial = true;
            this.context = new WeakReference<>(context);
        }

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
         * Set writer for data logging.
         * @param writer
         */
        public void setLogWriter(Writer writer) {
            mLogWriter = writer;
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

        public void setInitialPosition(IndoorPosition position) {
            mInitialPosition = position;
        }

        /**
         * @return A ready-to-use startable AndroidIndoorNavigator or null if the builder is not properly configured.
         */
        public IndoorNavigator create() {
            // Check if there's everything
            if(mSources != null && mUpdater != null && mInitialPosition != null) {

                // Scan available sources
                AccelerometerHandler acc = null;
                GyroscopeHandler gyro = null;
                MagneticFieldHandler mag = null;
                WifiScanner wifi = null;
                for (StartableStoppable s : mSources) {
                    Class<?> type = s.getClass();
                    Log.d("ITERATION", "Current class is " + type.getName());
                    if (type == AccelerometerHandler.class) {
                        acc = (AccelerometerHandler) s;
                        if (mLogWriter != null)
                            acc.register(new Logger<Acceleration>(mLogWriter));
                    } else if (type == GyroscopeHandler.class) {
                        gyro = (GyroscopeHandler) s;
                        if(mLogWriter != null)
                            gyro.register(new Logger<AngularSpeed>(mLogWriter));
                    } else if (type == MagneticFieldHandler.class) {
                        mag = (MagneticFieldHandler) s;
                        if(mLogWriter != null)
                            mag.register(new Logger<MagneticField>(mLogWriter));
                    } else if (type == WifiScanner.class) {
                        wifi = (WifiScanner) s;
                        if(mLogWriter != null)
                            wifi.register(new Logger<WifiFingerprint>(mLogWriter));
                    }
                }
                Log.d("SCANRESULT", "sources empty? " + mSources.isEmpty() + " | " +
                        "acc" + acc + ", gyro " + gyro + ", mag " + mag + ", wifi " + wifi);

                // Wifi Fingerprint
                KnnWifiFingerprint wifiLoc = null;
                if (wifi != null && mWifiFingerprintMap != null) {
                    // Build locator instance
                    wifiLoc = KnnWifiFingerprint.makeInstance(
                            mWifiFingerprintMap, 0, WIFI_FINGERPRINT_THRESHOLD);

                    // Register locator to wifi emitter
                    wifi.register(wifiLoc);

                    // Register logger to wifi location
                    if(mWifiFingerprintPositionLogger != null)
                        wifiLoc.register(mWifiFingerprintPositionLogger);
                }

                // Geomagnetic fingerprint
                KnnMagneticMismatch mm = null;
                if (mag != null && mMagneticFingerprintMap != null) {
                    // Build locator instance
                    mm = KnnMagneticMismatch.makeInstance(
                            mMagneticFingerprintMap, 0, MM_KNN, MM_THRESHOLD);
                    // Register locator to magnetic emitter
                    mag.register(mm);
                    // Register logger to mm location
                    if(mMMPositionLogger != null)
                        mm.register(mMMPositionLogger);
                }

                // Choose strategy and link everything
                if (acc != null && mag != null && gyro != null) {
                    Log.d("INBUILDER","PDR recognised");

                    // Heading
                    final Compass heading = new RelativeCompass(acc,gyro,mag,60);
                    if(builderIsSocial) {
                        // Observer for first-heading notification: when it occurs,
                        // compass is calibrated and active.
                        // Indeed, RelativeCompass doesn't send heading notification until it has done
                        // with calibration.
                        final Observer<Heading> calibrationNotificator = new Observer<Heading>() {

                            private boolean done = false;

                            @Override
                            public void notify(Heading data) {
                                if(!done) {
                                    Log.d("COMPASSCAL","HEADING!");
                                    Context c;
                                    if ((c = context.get()) != null)
                                        Toast.makeText(c,
                                                c.getString(R.string.compass_calibration_end),
                                                Toast.LENGTH_SHORT)
                                                .show();
                                    // Start navigator after compass calibration
                                    nav.start();
                                    // Do this only at first heading data received
                                    done = true;
                                }
                            }
                        };
                        heading.register(calibrationNotificator);
                    }

                    // Step detection
                    StepDetector sd = new FasterStepDetector(acc);

                    // Pseudo-You Li strategy
                    FixedLengthPDR pdr = new FixedLengthPDR(heading,sd,0.f);

                    // Pseudo-You Li strategy with KF
                    Log.d("BUILDER", "Initial position is " + mInitialPosition);
                    mStrategy = new SimpleKalmanFilterStrategy(
                            mInitialPosition,
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

                // Initialize AndroidIndoorNavigator instance
                nav = new AndroidIndoorNavigator(mSources);
                if(gyro != null && mStrategy != wifiLoc && mStrategy != mm) {
                    nav.stop();
                    gyro.start();
                } else
                    nav.start();

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
