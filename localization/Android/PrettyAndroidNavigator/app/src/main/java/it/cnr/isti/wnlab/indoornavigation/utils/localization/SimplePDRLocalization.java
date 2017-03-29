package it.cnr.isti.wnlab.indoornavigation.utils.localization;

import it.cnr.isti.wnlab.indoornavigation.javaonly.AbstractIndoorLocalizationStrategy;
import it.cnr.isti.wnlab.indoornavigation.javaonly.IndoorPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.XYPosition;
import it.cnr.isti.wnlab.indoornavigation.javaonly.map.FloorMap;
import it.cnr.isti.wnlab.indoornavigation.javaonly.observer.Observer;
import it.cnr.isti.wnlab.indoornavigation.javaonly.pdr.PDR;

public class SimplePDRLocalization extends AbstractIndoorLocalizationStrategy implements Observer<PDR.Result> {

    private XYPosition position;
    private PDR pdr;
    private FloorMap floor;

    public SimplePDRLocalization(XYPosition initialPosition, PDR pdr, FloorMap floor) {
        this.position = initialPosition;
        this.pdr = pdr;
        this.floor = floor;
    }

    @Override
    public IndoorPosition getCurrentPosition() {
        return new IndoorPosition(position,floor.getFloor(),System.currentTimeMillis());
    }

    @Override
    protected void start() {
        pdr.register(this);
    }

    @Override
    protected void stop() {
        pdr.unregister(this);
    }

    @Override
    public void notify(PDR.Result data) {
        float x = position.x + data.dE;
        float y = position.y + data.dN;
        XYPosition p;
        if(floor != null)
            p = floor.nearestValid(x,y);
        else
            p = new XYPosition(x,y);
        notifyObservers(new IndoorPosition(p,floor.getFloor(),System.currentTimeMillis()));
    }
}
