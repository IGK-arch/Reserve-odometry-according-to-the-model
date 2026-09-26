"""Read/sample the map and reproduce the judge's fixed CB output transform.

The map follows GNSS master in fixed WGS84 ENU metres. The organiser clarified
that the target is base_link in fixed MGRS-CB numerical coordinates, with master
at body x=-9.873 m and z=+3 m relative to base_link.
"""

from __future__ import annotations

import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path


ORIGIN_LON_DEG = 37.462266845
ORIGIN_LAT_DEG = 55.810367065
ORIGIN_ALT_M = 168.3794
_A = 6378137.0
_E2 = 6.6943799901413165e-3


def _ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    radius = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    return ((radius + alt_m) * cos_lat * cos_lon,
            (radius + alt_m) * cos_lat * sin_lon,
            (radius * (1.0 - _E2) + alt_m) * sin_lat)


_OX, _OY, _OZ = _ecef(ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_ALT_M)
_LAT0 = math.radians(ORIGIN_LAT_DEG)
_LON0 = math.radians(ORIGIN_LON_DEG)


def geodetic_to_enu(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    """WGS84 geodetic → ECEF → East/North/Up at the fixed project datum."""
    x, y, z = _ecef(lat_deg, lon_deg, alt_m)
    dx, dy, dz = x - _OX, y - _OY, z - _OZ
    slon, clon = math.sin(_LON0), math.cos(_LON0)
    slat, clat = math.sin(_LAT0), math.cos(_LAT0)
    return (-slon * dx + clon * dy,
            -slat * clon * dx - slat * slon * dy + clat * dz,
            clat * clon * dx + clat * slon * dy + slat * dz)


def enu_to_geodetic(east_m: float, north_m: float, up_m: float) -> tuple[float, float, float]:
    """Fixed project ENU → WGS84 latitude, longitude, ellipsoid altitude.

    ``pyproj`` is an offline analysis dependency only. The ROS executable has
    its own C++ transform and does not import this helper.
    """
    from pyproj import Transformer

    slon, clon = math.sin(_LON0), math.cos(_LON0)
    slat, clat = math.sin(_LAT0), math.cos(_LAT0)
    dx = -slon * east_m - slat * clon * north_m + clat * clon * up_m
    dy = clon * east_m - slat * slon * north_m + clat * slon * up_m
    dz = clat * north_m + slat * up_m
    longitude, latitude, altitude = Transformer.from_crs(
        4978, 4979, always_xy=True
    ).transform(_OX + dx, _OY + dy, _OZ + dz)
    return latitude, longitude, altitude


def geodetic_to_fixed_cb(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    """WGS84 → judge's numeric MGRS-CB coordinates in metres.

    The organiser's concrete example matches UTM zone 37N (EPSG:32637) with
    fixed grid-square offsets easting 300000 m and northing 6100000 m. These
    are continuous x/y numbers, not an alphanumeric MGRS string.
    """
    from pyproj import Transformer

    easting, northing, _ = Transformer.from_crs(
        4979, 32637, always_xy=True
    ).transform(lon_deg, lat_deg, alt_m)
    return easting - 300000.0, northing - 6100000.0, alt_m


def enu_to_fixed_cb(east_m: float, north_m: float, up_m: float) -> tuple[float, float, float]:
    """Project an internal ENU map point to judge CB numeric x/y/z."""
    latitude, longitude, altitude = enu_to_geodetic(east_m, north_m, up_m)
    return geodetic_to_fixed_cb(latitude, longitude, altitude)


@dataclass(frozen=True)
class Projection:
    direction: str
    s: float
    x: float
    y: float
    z: float
    horizontal_distance_m: float


class RouteMap:
    def __init__(self, rows: dict[str, list[tuple[float, float, float, float]]]):
        self.rows = rows
        self.abscissae = {key: [row[0] for row in value] for key, value in rows.items()}
        for direction in ("out", "return"):
            data = rows.get(direction, [])
            if len(data) < 2 or abs(data[0][0]) > 1e-6 or any(b[0] <= a[0] for a, b in zip(data, data[1:])):
                raise ValueError(f"Invalid or missing route direction {direction}")

    @classmethod
    def from_csv(cls, path: str | Path) -> "RouteMap":
        rows: dict[str, list[tuple[float, float, float, float]]] = {"out": [], "return": []}
        with Path(path).open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames != ["direction", "s", "x", "y", "z"]:
                raise ValueError("Expected CSV columns direction,s,x,y,z")
            for row in reader:
                direction = row["direction"]
                if direction not in rows:
                    raise ValueError(f"Unknown direction {direction}")
                rows[direction].append(tuple(float(row[field]) for field in ("s", "x", "y", "z")))
        return cls(rows)

    def sample(self, direction: str, s: float) -> tuple[float, float, float]:
        """Linear interpolation at arc coordinate, clamped to route endpoints."""
        data = self.rows[direction]
        abscissa = self.abscissae[direction]
        if s <= 0:
            return data[0][1:]
        if s >= abscissa[-1]:
            return data[-1][1:]
        i = bisect.bisect_right(abscissa, s)
        before, after = data[i - 1], data[i]
        a = (s - before[0]) / (after[0] - before[0])
        return tuple(before[j] + a * (after[j] - before[j]) for j in (1, 2, 3))

    def project(self, x: float, y: float, direction: str | None = None) -> Projection:
        """Project an initial 2D GNSS point on the nearest direction/segment."""
        best: Projection | None = None
        directions = [direction] if direction else ["out", "return"]
        for current in directions:
            points = self.rows[current]
            for a, b in zip(points, points[1:]):
                dx, dy = b[1] - a[1], b[2] - a[2]
                length_sq = dx * dx + dy * dy
                if length_sq < 1e-8:
                    continue
                fraction = min(1.0, max(0.0, ((x - a[1]) * dx + (y - a[2]) * dy) / length_sq))
                px = a[1] + fraction * dx
                py = a[2] + fraction * dy
                d = math.hypot(x - px, y - py)
                if best is None or d < best.horizontal_distance_m:
                    best = Projection(current, a[0] + fraction * (b[0] - a[0]),
                                      px, py, a[3] + fraction * (b[3] - a[3]), d)
        if best is None:
            raise ValueError("No route segment")
        return best

    def master_to_base_enu(self, direction: str, s: float,
                           forward_m: float = 9.873,
                           antenna_height_m: float = 3.0) -> tuple[float, float, float]:
        """Apply organiser TF master=(x=-9.873,z=3) relative to base_link.

        The online C++ node uses a rigid body transform along the local 3D
        tangent at ``s``. This differs from advancing 9.873 m along the rail
        on curves. Train GNSS pairs verify the forward sign both ways.
        """
        # An odometry overrun is clamped by both the C++ map reader and this
        # helper. Clamp *before* finding the tangent so s±1 still spans the
        # final/initial segment rather than becoming two identical endpoints.
        s_clamped = min(max(0.0, s), self.abscissae[direction][-1])
        p = self.sample(direction, s_clamped)
        before = self.sample(direction, s_clamped - 1.0)
        after = self.sample(direction, s_clamped + 1.0)
        tangent = tuple(after[i] - before[i] for i in range(3))
        norm = math.sqrt(sum(value * value for value in tangent))
        if norm < 1e-8:
            raise ValueError("Degenerate route tangent")
        return (p[0] + forward_m * tangent[0] / norm,
                p[1] + forward_m * tangent[1] / norm,
                p[2] + forward_m * tangent[2] / norm - antenna_height_m)

    def master_to_base_fixed_cb(self, direction: str, s: float) -> tuple[float, float, float]:
        """Offline equivalent of map master → base_link → judge CB transform."""
        return enu_to_fixed_cb(*self.master_to_base_enu(direction, s))


if __name__ == "__main__":
    sample_lat = 55.8088325462547
    sample_lon = 37.4602768500852
    enu = geodetic_to_enu(sample_lat, sample_lon, ORIGIN_ALT_M)
    fixed = enu_to_fixed_cb(*enu)
    assert abs(fixed[0] - 103501.6309) < 0.001, fixed
    assert abs(fixed[1] - 85876.1201) < 0.001, fixed
    print("CB numeric example reproduced:", fixed)
