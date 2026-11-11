"""Pattern 3, contracting spiral: flies the outer edge first, then each lap further in.

Run:
    ros2 run gnc_offboard mission_spiral_in
"""
from gnc_offboard import search_planners as sp
from gnc_offboard.search_base import SearchMission, run


class SpiralInMission(SearchMission):
    PATTERN = 'contracting_spiral'

    def plan(self, area, start):
        return sp.contracting_spiral(area, start, self.spacing)


def main(args=None):
    run(SpiralInMission, args)


if __name__ == '__main__':
    main()
