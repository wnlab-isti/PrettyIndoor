package it.cnr.isti.wnlab.indoornavigation.android.graphics;

import android.graphics.Canvas;
import android.graphics.Paint;

import java.util.Collection;

import it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter.PositionParticle;

public class ParticlesDrawable implements IndoorDrawable {

    private Collection<PositionParticle> particles;
    private Paint paint;
    private float radius;

    public ParticlesDrawable(Collection<PositionParticle> particles, int color) {
        // Set particles
        this.particles = particles;
        // Set width and height
        radius = 0.3f;
        // Instantiate paint object
        this.paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        this.paint.setColor(color);
        this.paint.setStyle(Paint.Style.FILL);
    }

    @Override
    public void draw(Canvas canvas) {
        for(PositionParticle p : particles)
            canvas.drawCircle(p.getX(),p.getY(),radius,paint);
    }
}
