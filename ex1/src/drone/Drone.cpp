#include "drone/Drone.h"

namespace drone {

Drone::Drone(ILidarSensor&    lidar,
             IPositionSensor& position,
             IMovementDriver& driver,
             IBuildingMap&    map)
    : m_lidar(lidar)
    , m_position(position)
    , m_driver(driver)
    , m_map(map)
{}

MoveResult Drone::Rotate(HorizontalAngle angle)  { return m_driver.Rotate(angle); }
MoveResult Drone::Advance(PhysicalLength distance) { return m_driver.Advance(distance); }
MoveResult Drone::Elevate(PhysicalLength distance) { return m_driver.Elevate(distance); }

ScanResults Drone::Scan(Orientation scan_orientation)
{
    return m_lidar.scan(scan_orientation);
}

Position3D Drone::GetLocation() const
{
    return m_position.position();
}

void Drone::RecordCell(XLength x, YLength y, ZLength z, MapValue value)
{
    m_map.Set(x, y, z, value);
}

MapValue Drone::QueryCell(XLength x, YLength y, ZLength z) const
{
    return m_map.Get(x, y, z);
}

} // namespace drone
