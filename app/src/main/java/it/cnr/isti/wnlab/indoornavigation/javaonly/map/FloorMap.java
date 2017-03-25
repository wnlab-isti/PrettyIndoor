package it.cnr.isti.wnlab.indoornavigation.javaonly.map;

import java.util.Arrays;
import java.util.Collection;

public class FloorMap implements XYMap {

    private int floor;
    private Collection<RoomMap> rooms;

    public FloorMap(int floor, RoomMap... rooms) {
        this.floor = floor;
        this.rooms = Arrays.asList(rooms);
    }

    @Override
    public boolean isValid(float x, float y) {
        for(RoomMap room : rooms) {
            if(room.isValid(x,y))
                return true;
        }
        return false;
    }

    public int getFloor() {
        return floor;
    }
}
