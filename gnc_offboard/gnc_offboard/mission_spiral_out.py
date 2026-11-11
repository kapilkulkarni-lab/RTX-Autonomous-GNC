"""Pattern 2, expanding spiral: searches near the start zone first, then works outward.

The start zone is in a corner, so the "spiral" is a set of expanding L-shaped rings
clipped to the field, flown back and forth.

Run:
    ros2 run gnc_offboard mission_spiral_out
"""
from gnc_offboard import search_planners as sp
from gnc_offboard.search_base import SearchMission, run


class SpiralOutMission(SearchMission):
    PATTERN = 'expanding_spiral'

    def plan(self, area, start):
        return sp.expanding_spiral(area, start, self.spacing)


def main(args=None):
    run(SpiralOutMission, args)


if __name__ == '__main__':
    main()
