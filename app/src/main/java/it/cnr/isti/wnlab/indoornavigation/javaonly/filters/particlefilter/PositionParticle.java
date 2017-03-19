package it.cnr.isti.wnlab.indoornavigation.javaonly.filters.particlefilter;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Random;

import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;

public class PositionParticle implements Particle {

    private float x,y,weight;

    public PositionParticle(float x, float y, float weight) {
        this.x = x;
        this.y = y;
        this.weight = weight;
    }

    public float getX() {
        return x;
    }

    public void setX(float x) {
        this.x = x;
    }

    public float getY() {
        return y;
    }

    public void setY(float y) {
        this.y = y;
    }

    public float getWeight() {
        return weight;
    }

    public void setWeight(float weight) {
        this.weight = weight;
    }

    /**
     * @return The position this particle represents.
     */
    public XYPosition toPosition() {
        return new XYPosition(x,y);
    }

    /**
     * @param n The number of particles.
     * @param minX Minimum x coordinate.
     * @param minY Minimum y coordinate.
     * @param maxX Maximum x coordinate.
     * @param maxY Maximum y coordinate.
     * @return A collection of n equally-weighted particles with gaussian-distributed coordinates.
     */
    public static Collection<PositionParticle> getGaussianCollection(int n, float minX, float minY, float maxX, float maxY) {
        Random rx = new Random();
        Random ry = new Random();
        ArrayList<PositionParticle> particles = new ArrayList<>();
        for(int i = 0; i < n; i++) {
            float x = ((float) rx.nextGaussian())*(maxX-minX)+minX;
            float y = ((float) ry.nextGaussian())*(maxY-minY)+minY;
            particles.add(new PositionParticle(x,y,1.f/n));
        }
        return particles;
    }
}
