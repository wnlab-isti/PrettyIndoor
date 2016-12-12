package it.cnr.isti.wnlab.indoornavigator.androidutils;

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
import it.cnr.isti.wnlab.indoornavigator.framework.util.strategy.KFUleeStrategy;

/**
 * IndoorNavigator object for the user.
 */
public class IndoorNavigator implements StartableStoppable {

    private List<StartableStoppable> mSources;
    private LocationStrategy mStrategy;
    private PositionUpdateCallback mUpdater;

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
                    KFUleeStrategy strategy = new KFUleeStrategy(
                            new IndoorPosition(0.f, 0.f, 0, System.currentTimeMillis()),
                            0.f,
                            mUpdater);

                    // Heading from sensor fusion
                    Compass heading = new LawitzkiCompass(strategy, acc, gyro, mag, 30);
                    sources.add(heading);

                    // Step detection
                    StepDetector sd = new FasterStepDetector(strategy, acc);
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
