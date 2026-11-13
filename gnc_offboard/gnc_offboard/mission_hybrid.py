"""Fixed hybrid pattern: coarse alternating rows, followed by a fill pass.

Camera detections are recorded independently and never change this path.
Run: ros2 run gnc_offboard mission_hybrid
"""
from gnc_offboard import search_planners as sp
from gnc_offboard.search_base import SearchMission, run


class HybridMission(SearchMission):
    PATTERN = 'hybrid'

    def plan(self, area, start):
        return sp.hybrid_sweep(area, start, self.spacing)


def main(args=None):
    run(HybridMission, args)


if __name__ == '__main__':
    main()
