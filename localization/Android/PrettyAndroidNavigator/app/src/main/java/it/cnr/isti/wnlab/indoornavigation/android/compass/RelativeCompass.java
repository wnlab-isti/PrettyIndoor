package it.cnr.isti.wnlab.indoornavigation.android.compass;

import android.util.Log;

import it.cnr.isti.wnlab.indoornavigation.observer.DataEmitter;
import it.cnr.isti.wnlab.indoornavigation.types.inertial.Acceleration;
import it.cnr.isti.wnlab.indoornavigation.types.inertial.AngularSpeed;
import it.cnr.isti.wnlab.indoornavigation.types.environmental.MagneticField;

/**
 * «FOR THE NORTH!»
 * This class finds the current orientation and sets it as the north (zero-valued).
 * For example, if I want to set the heading "zero" as the east of a map, I just have to orientate
 * myself to that direction and let it calibrate.
 * <p>
 * The behaviour of this class has two phases:
 * 1) Calibration (NOTE: this is not the usual calibration for compasses in smartphones)
 *      It doesnt' notify heading observers. It uses LawitzkiCompass's updates to find the
 *      orientation. When this has stabilized in one direction, saves it to correct future headings.
 * 2) Heading emission
 *      After calibration, starts emitting a corrected heading with values from -pi to pi, value 0
 *      at the saved orientation during previous phase.
 *      The value is greater if rotating counter-clockwise and minor if rotating clockwise (as in
 *      the unit circle).
 * <p>
 * Note that this starts emitting heading only after stabilizing in a direction.
 *
 * @author Michele Agostini
 */
public class RelativeCompass extends LawitzkiCompass {

    // Constants
    private final static float ONE_DEGREE_IN_RADIANTS = (float) Math.toRadians(1.f);
    private final static float TOLERANCE = 10.f * ONE_DEGREE_IN_RADIANTS;
    private final static int CALIBRATION_COUNTER_MAX = 80;
    private final static int CALIBRATION_RATE = 60;
    private final static int OPERATIVE_RATE = 500;

    // Variables for calibration
    private boolean calibration = true;
    private int counter;
    private float lastCalibrationHeading;

    public RelativeCompass(
            DataEmitter<Acceleration> accelerometer,
            DataEmitter<AngularSpeed> gyroscope,
            DataEmitter<MagneticField> magnetometer
    ) {
        super(accelerometer, gyroscope, magnetometer, CALIBRATION_RATE);
    }

    @Override
    protected void onHeadingChange(float newHeading, long timestamp) {
        if(calibration) {
            // Log.d("COMPASS", "Finding heading zero: " + counter);
            // Check if newHeading is legal for counting (belongs to [H-1; H+1])
            if(lastCalibrationHeading < (newHeading - TOLERANCE) || lastCalibrationHeading > (newHeading + TOLERANCE)) {
                // Reset counter
                counter = 0;
                // Replace last heading
                lastCalibrationHeading = newHeading;
            } else
                counter++;

            // Calibrate until calibration count max is reached
            if(counter == CALIBRATION_COUNTER_MAX) {
                calibration = false;
                setRate(OPERATIVE_RATE);
            }
        } else {
            // If not calibrating, update heading with correction
            float correctHeading = newHeading-lastCalibrationHeading;
            if(correctHeading < -Math.PI)
                correctHeading += 2.f*Math.PI;
            else if(correctHeading > Math.PI)
                correctHeading -= 2.f*Math.PI;
            correctHeading *= -1;
            // Notify to observers correct heading
            super.onHeadingChange(correctHeading, timestamp);
            Log.d("COMPASS", "heading: " + correctHeading);
        }
    }
}
