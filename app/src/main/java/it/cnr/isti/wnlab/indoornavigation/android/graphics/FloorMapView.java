package it.cnr.isti.wnlab.indoornavigation.android.graphics;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.view.View;

import java.util.ArrayList;
import java.util.Collection;

public class FloorMapView extends View {

    private Matrix cartesian2computing;
    //private Matrix computing2cartesian;
    private Collection<IndoorDrawable> drawables;

    public FloorMapView(Context context, float pixelPerMeter, float northBound) {
        super(context);

        // Initialize transformation matrices
        this.cartesian2computing = new Matrix();
        //this.computing2cartesian = new Matrix();

        // y-coordinate correction
        cartesian2computing.postTranslate(0.f,-northBound);
        //computing2cartesian.preTranslate(0.f, northBound);

        // Meter-pixel scaling
        cartesian2computing.postScale(1.f/pixelPerMeter,1.f/pixelPerMeter);
        //computing2cartesian.preScale(pixelPerMeter,pixelPerMeter);

        // Initialize drawables collection
        drawables = new ArrayList<>();
    }

    public boolean addDrawable(IndoorDrawable drawable) {
        return this.drawables.add(drawable);
    }

    public boolean addDrawables(Collection<IndoorDrawable> drawables) {
        return this.drawables.addAll(drawables);
    }

    public boolean removeDrawable(IndoorDrawable drawable) {
        return this.drawables.remove(drawable);
    }

    public boolean removeDrawables(Collection<IndoorDrawable> drawables) {
        return this.drawables.removeAll(drawables);
    }

    @Override
    public void onDraw(Canvas canvas) {
        // Prepare matrix for drawing "meters"
        canvas.save();
        canvas.setMatrix(cartesian2computing);

        // Draw all IndoorDrawables
        for(IndoorDrawable d : drawables)
                d.draw(canvas);

        // Restore graphic context
        canvas.restore();
    }

}
