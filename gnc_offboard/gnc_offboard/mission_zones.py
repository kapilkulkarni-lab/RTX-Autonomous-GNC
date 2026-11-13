"""Pattern 5, zone / sector: split the field into zones, visit by priority, sweep each.

Priority = weight / (1 + distance to the zone center), so equal weights means "nearest
zone next". For a replan (Challenge 3) pass `cleared` with the zones already searched.

Zones are numbered row by row from the bottom-left (x east, y north): with 3 x 3,
0 1 2 is the south row and 6 7 8 is the north row.

Run:
    ros2 run gnc_offboard mission_zones
    ros2 run gnc_offboard mission_zones --ros-args -p zones_x:=4 -p zones_y:=4
    ros2 run gnc_offboard mission_zones --ros-args -p cleared:="[0, 1]" \
        -p zone_weights:="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 5.0]"
"""
from rclpy.parameter import Parameter

from gnc_offboard import search_planners as sp
from gnc_offboard.search_base import SearchMission, run


class ZoneMission(SearchMission):
    PATTERN = 'zone_sweep'
    PARAMS = {'zones_x': 3, 'zones_y': 3}

    def declare_extra_parameters(self):
        self.declare_parameter('zone_weights', Parameter.Type.DOUBLE_ARRAY)   # one per zone
        self.declare_parameter('cleared', Parameter.Type.INTEGER_ARRAY)       # zones to skip

    def plan(self, area, start):
        weights = self.optional_array('zone_weights')
        cleared = self.optional_array('cleared') or []
        return sp.zone_sweep(area, start, self.spacing, self.pp['zones_x'], self.pp['zones_y'],
                             weights=weights, cleared=cleared, pad=self.margin)

    def plan_region(self, area, start):
        return self.plan(area, start) or None


def main(args=None):
    run(ZoneMission, args)


if __name__ == '__main__':
    main()
