"""Pattern 4, frontier-based: always fly toward the nearest area not yet seen.

This version plans the whole path before takeoff, assuming the camera sees exactly its
footprint. On the real UAV the same "seen / unseen" map would be updated online from the
camera, and the next frontier chosen after each leg.

Run:
    ros2 run gnc_offboard mission_frontier
    ros2 run gnc_offboard mission_frontier --ros-args -p cell:=0.4 -p goal:=0.99
"""
from gnc_offboard import search_planners as sp
from gnc_offboard.search_base import SearchMission, run


class FrontierMission(SearchMission):
    PATTERN = 'frontier'
    PARAMS = {'cell': 0.5,      # m, size of one coverage-grid square
              'goal': 0.995}    # stop when this fraction of the field has been seen

    def plan(self, area, start):
        return sp.frontier(area, start, self.fp, pad=self.margin, cell=self.pp['cell'],
                           obstacles=self.obstacles, clearance=self.get_parameter('clearance').value,
                           goal=self.pp['goal'])

    def plan_region(self, area, start):
        return sp.frontier(area, start, self.fp, cell=self.pp['cell'], goal=self.pp['goal'])


def main(args=None):
    run(FrontierMission, args)


if __name__ == '__main__':
    main()
