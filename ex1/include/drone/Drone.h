#pragma once

#include "interfaces/ILidarSensor.h"
#include "interfaces/IPositionSensor.h"
#include "interfaces/IMovementDriver.h"
#include "interfaces/IBuildingMap.h"
#include "types/Units.h"
#include "types/MapValue.h"

namespace drone {

class Drone {
public:
    Drone(ILidarSensor&    lidar,
          IPositionSensor& position,
          IMovementDriver& driver,
          IBuildingMap&    map);

    MoveResult  Rotate(HorizontalAngle angle);
    MoveResult  Advance(PhysicalLength distance);
    MoveResult  Elevate(PhysicalLength distance);

    ScanResults Scan(Orientation scan_orientation = {});

    Position3D  GetLocation() const;

    void        RecordCell(XLength x, YLength y, ZLength z, MapValue value);
    MapValue    QueryCell(XLength x, YLength y, ZLength z) const;

private:
    ILidarSensor&    m_lidar;
    IPositionSensor& m_position;
    IMovementDriver& m_driver;
    IBuildingMap&    m_map;
};

} // namespace drone
