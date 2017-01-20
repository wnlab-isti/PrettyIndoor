package it.cnr.isti.wnlab.indoornavigator.android.compass;

import android.util.Log;

import it.cnr.isti.wnlab.indoornavigator.emitters.Emitter;
import it.cnr.isti.wnlab.indoornavigator.types.inertial.Acceleration;
import it.cnr.isti.wnlab.indoornavigator.types.inertial.AngularSpeed;
import it.cnr.isti.wnlab.indoornavigator.types.Heading;
import it.cnr.isti.wnlab.indoornavigator.types.environmental.MagneticField;

/**
 * FOR THE NORTH! (Elaborates an initial direction to refer to as the North.)
 */
public class RelativeCompass extends LawitzkiCompass {

    // Constants
    private final static float ONE_DEGREE_IN_RADIANTS = (float) Math.toRadians(1.f);
    private final static float TOLERANCE = 10.f * ONE_DEGREE_IN_RADIANTS;
    private final static int CALIBRATION_COUNTER_MAX = 80;

    // Variables for calibration
    private boolean calibration = true;
    private int counter;
    private float lastCalibrationHeading;

    public RelativeCompass(
            Emitter<Acceleration> accelerometer,
            Emitter<AngularSpeed> gyroscope,
            Emitter<MagneticField> magnetometer,
            int rate
    ) {
        super(accelerometer, gyroscope, magnetometer, rate);
    }

    @Override
    protected void onHeadingChange(float newHeading, long timestamp) {
        if(calibration) {
            Log.d("COMPASSCAL", "heading: " + counter);
            // Check if newHeading is legal for counting (belongs to [H-1; H+1])
            if(lastCalibrationHeading < (newHeading - TOLERANCE) || lastCalibrationHeading > (newHeading + TOLERANCE)) {
                // Reset counter
                counter = 0;
                // Replace last heading
                lastCalibrationHeading = newHeading;
            } else
                counter++;

            // Calibrate until calibration count max is reached
            if(counter == CALIBRATION_COUNTER_MAX)
                calibration = false;
        } else {
            // If not calibrating, update heading with correction
            float correctHeading = newHeading-lastCalibrationHeading;
            notifyObservers(new Heading(correctHeading,timestamp));
        }
    }


}
