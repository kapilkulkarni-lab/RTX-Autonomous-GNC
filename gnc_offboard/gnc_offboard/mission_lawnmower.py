"""Pattern 1, lawnmower: parallel rows across the field, starting at the start-zone corner.

Run (sim + agent already up, drone landed and disarmed):
    ros2 run gnc_offboard mission_lawnmower
    ros2 run gnc_offboard mission_lawnmower --ros-args -p rows_along:=y -p fov_deg:=80.0
"""
from gnc_offboard import search_planners as sp
from gnc_offboard.search_base import SearchMission, run


class LawnmowerMission(SearchMission):
    PATTERN = 'lawnmower'
    PARAMS = {'rows_along': 'x'}      # 'x' or 'y': which way the long rows run

    def plan(self, area, start):
        return sp.lawnmower(area, start, self.spacing, self.pp['rows_along'])


def main(args=None):
    run(LawnmowerMission, args)


if __name__ == '__main__':
    main()
