"""Shared color scale for live and saved sensing-time maps."""

DARKEST_AT_S = 30


def sensing_color(seconds):
    """Light orange at first sighting, dark brown at 30 accumulated seconds."""
    fraction = min(seconds / DARKEST_AT_S, 1.0)
    return (1.0 - .54 * fraction, .71 - .53 * fraction, .29 - .24 * fraction)
