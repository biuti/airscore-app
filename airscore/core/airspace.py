from dataclasses import dataclass, asdict, fields
from airspaceUtils import read_airspace_check_file
from Defines import AIRSPACEDIR, AIRSPACEMAPDIR, AIRSPACECHECKDIR
from pathlib import Path
from os import path as p


@dataclass(frozen=True)
class CheckParams:
    notification_distance: int = 100
    outer_limit: int = 70
    border_penalty: float = 0.1
    inner_limit: int = -30
    max_penalty: float = 1.0
    outer_penalty_per_m = border_penalty / outer_limit
    inner_penalty_per_m = max_penalty if inner_limit == 0 else (max_penalty - border_penalty) / abs(inner_limit)

    def penalty(self, distance) -> float:
        if distance >= self.inner_limit:
            return 0
        elif distance <= self.inner_limit:
            return 1.0
        elif distance >= 0:
            return (self.outer_limit - distance) * self.outer_penalty_per_m
        elif distance < 0:
            return self.border_penalty + abs(distance * self.inner_penalty_per_m)


class AirspaceCheck(object):
    def __init__(self, control_zones=None, params=None):
        self.control_zones = control_zones  # igc_lib openair reader control zones
        self.params = params  # AirspaceCheck object
        self.get_airspace_details()

    @property
    def bounding_box(self):
        if self.control_zones:
            return self.control_zones['bbox']

    @property
    def airspace_details(self):
        if self.control_zones:
            if not any(space['object'] for space in self.control_zones['spaces']):
                self.get_airspace_details()
            return self.control_zones['spaces']

    @property
    def projection(self):
        """WGS84 to Mercatore Plan Projection"""
        from pyproj import Proj, Transformer
        '''get projection center'''
        clat = self.bounding_box[0][0] + (self.bounding_box[1][0] - self.bounding_box[0][0])
        clon = self.bounding_box[0][1] + (self.bounding_box[1][1] - self.bounding_box[0][1])
        '''define earth model'''
        wgs84 = Proj("EPSG:4326")  # LatLon with WGS84 datum used by GPS units and Google Earth
        tmerc = Proj(f"+proj=tmerc +lat_0={clat} +lon_0={clon} +k_0=1 +x_0=0 +y_0=0 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs")
        return Transformer.from_proj(wgs84, tmerc)

    @staticmethod
    def from_task(task):
        if task.airspace_check and task.openair_file:
            task_id = task.task_id
            control_zones = read_airspace_check_file(task.openair_file)
            params = get_airspace_check_parameters(task_id)
            return AirspaceCheck(control_zones, params)

    @staticmethod
    def read(task_id):
        from task import Task
        try:
            task = Task.read(task_id)
            return AirspaceCheck.from_task(task)
        except:
            print(f'Error trying to get task control zones')

    def get_airspace_details(self):
        from route import Turnpoint
        from shapely.geometry import Point
        from shapely.geometry.polygon import Polygon
        import numpy as np

        if self.control_zones:
            for space in self.control_zones['spaces']:
                if space['shape'] == 'circle':
                    space['object'] = Turnpoint(lat=space['location'][0], lon=space['location'][1],
                                                radius=space['radius'])
                elif space['shape'] == 'polygon':
                    # TODO we get an error if airspace is an Arc. Transforming to circle with radius 1000m
                    if len(space['locations']) == 1:
                        # Arc
                        space['object'] = Turnpoint(lat=space['locations'][0][0], lon=space['locations'][0][1],
                                                    radius=1000)
                    else:
                        space['object'] = []
                        lats = []
                        lons = []
                        for p in space['locations']:
                            x, y = self.projection.transform(p[1], p[0])
                            lats.append(y)
                            lons.append(x)
                        lats_vect = np.array(lats)
                        lons_vect = np.array(lons)
                        lons_lats_vect = np.column_stack((lons_vect, lats_vect))
                        polygon = Polygon(lons_lats_vect)
                        space['object'] = polygon

    def check_fix(self, fix, altitude_mode='gps'):
        """check a flight object for airspace violations
        arguments:
        fix - Flight fix object
        altimeter - flight altitude to use in checking 'barometric' - barometric altitude,
                                                      'gps' - GPS altitude
                                                      'baro/gps' - barometric if present otherwise gps  (default)
        vertical_tolerance: vertical distance in meters that a pilot can be inside airspace without penalty (default 0)
        horizontal_tolerance: horizontal distance in meters that a pilot can be inside airspace without penalty (default 0)
        """
        from airspaceUtils import altitude, in_bbox
        from shapely.geometry import Point
        from route import distance

        # airspace_plot = []
        violation = False
        notification_band = self.params.notification_distance
        penalty_band = self.params.outer_limit

        alt = altitude(fix, altitude_mode)
        # fix_violation = False
        if in_bbox(self.bounding_box, fix):  # check if we are in the bounding box of all airspaces
            for space in self.airspace_details:
                # we are at same alt as an airspace
                if space['floor'] - notification_band < alt < space['ceiling'] + notification_band:
                    if space['shape'] == 'circle':
                        if space['object'].in_radius(fix, 0, notification_band):
                            '''We are inside Notification band'''
                            dist_floor = space['floor'] - alt
                            dist_ceiling = alt - space['ceiling']
                            dist_horiz = distance(fix, space['object']) - space['object'].radius
                            distance = max(dist_floor, dist_ceiling, dist_horiz)
                            infringement = space['name']
                    elif space['shape'] == 'polygon':
                        x, y = self.projection.transform(fix.lon, fix.lat)
                        point = Point(y, x)
                        if point.within(space['object']):
                            airspace_plot.append([fix.rawtime, fix.lat, fix.lon, alt, space['floor'],
                                                  space['ceiling'], space['name']])
                            violation = True
                            fix_violation = True
                    # TODO insert arc check here. we can use in radius and bearing to

        if not fix_violation:
            airspace_plot.append([fix.rawtime, fix.lat, fix.lon, alt, None, None])
        return airspace_plot, violation


def get_airspace_check_parameters(task_id):
    from myconn import Database
    from db_tables import TaskAirspaceCheckView as A
    from sqlalchemy.exc import SQLAlchemyError

    with Database() as db:
        try:
            q = db.session.query(A).get(task_id)
            if q.airspace_check:
                return CheckParams(q.notification_distance, q.outer_limit,
                                   q.border_penalty, q.inner_limit, q.max_penalty)
            else:
                return None
        except SQLAlchemyError:
            print(f'SQL Error trying to get Airspace Params from database')
            return None
