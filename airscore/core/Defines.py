"""
File created from install / settongs
Use: from Defines import BINDIR, TRACKDIR

Antonio Golfari - 2018
"""
import os
import yaml
from environs import Env
env = Env()
env.read_env()

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
with open('../../defines.yaml', 'rb') as f:
    """use safe_load instead load"""
    config = yaml.safe_load(f)

try:
    f = open('../../secret.yaml', 'rb')
    """use safe_load instead load"""
    secret = yaml.safe_load(f)
except IOError:
    secret = {}

os.chdir(dname)
''' Application Settings'''
# FLASKCONTAINER = config['docker']['container']  # Flask Docker Container Name
# FLASKPORT = config['docker']['port']  # Flask Docker Container Port
BINDIR = config['dir']['bin']  # script directory
TRACKDIR = config['dir']['tracks']  # track file directory
LOGDIR = config['dir']['log']  # log files directory
RESULTDIR = config['dir']['result']  # log files directory
IMAGEDIR = config['dir']['image']  # image/icon files directory
MAPOBJDIR = config['dir']['map']  # mapobj files directory
AIRSPACEDIR = config['dir']['airspace']  # openair files directory
AIRSPACEMAPDIR = config['dir']['airspace_map']  # airspace map files directory
AIRSPACECHECKDIR = config['dir']['airspace_check']  # airspace check files directory
WAYPOINTDIR = config['dir']['waypoint']  # waypoint files directory
LIVETRACKDIR = config['dir']['livetracking']  # waypoint files directory
IGCPARSINGCONFIG = config['dir']['igc_parsing_config']  # igc parsing config files
TEMPFILES = config['dir']['temp_files']  # tempfile folder when we need one that can be seen by other containers. e.g. workers

''' Track file Settings'''
track_sources = [s for s in config['igc_sources'] if config['igc_sources'][s]]  # external available sources for tracks
track_formats = ['igc']   # accepted track formats
'''accepted filename formats
id
name
civl
live
fai
other: other not used
examples: 
    '0068.igc' = 'id' 
    'LiveTrack Antoine Saraf.361951.20190717-113625.5836.47.igc' = 'other name name.live.other-other.other.id' '''
filename_formats = ['id', 'other name name.live.other-other.other.id', 'fai_name', 'name_name',
                    'other name name name.live.other-other.other.id',
                    'other name name name name.live.other-other.other.id',
                    'name_name.other-other.other.id',
                    'name_name.other-other.other.other',
                    'name_name_name.other-other.other.id',
                    'name_name_name_name.other-other.other.id',
                    'other name name.live.other-other.other.other',
                    ]

''' Waypoint file Settings'''
wpt_formats = ['GEO', 'UTM', 'CUP', 'GPX', 'CompeGPS', 'OziExplorer']
ALLOWED_WPT_EXTENSIONS = ['wpt', 'cup', 'gpx', 'ozi']

''' Database Settings'''
MYSQLUSER = secret.get('db', {}).get('User') or env.str('MYSQLUSER')  # mysql db user
MYSQLPASSWORD = secret.get('db', {}).get('Pass') or env.str('MYSQLPASSWORD')# mysql db password
MYSQLHOST = secret.get('db', {}).get('Server') or env.str('MYSQLHOST')   # mysql host name
DATABASE = secret.get('db', {}).get('Name') or env.str('DATABASE')# mysql db name

''' Other Settings'''
XC_LOGIN = secret.get('xcontest', {}).get('User') or env.str('XCONTEST_USER')
XC_password = secret.get('xcontest', {}).get('Pass') or env.str('XCONTEST_PASS')
G_Record_validation_Server = config['g_record_validation_server']

'''Competition options'''
SANCTIONS = config['sanctions']

'''File libraries'''
WAYPOINT_AIRSPACE_FILE_LIBRARY = config['waypoint/airspace_file_library']

'''Admin DB'''
ADMIN_DB = config['use_internal_admin_DB']
ADMIN_SELF_REG = config['internal_admin_DB']['allow_self_registration']


'''Pilot DB'''
PILOT_DB = config['use_internal_pilot_DB']
PILOT_DB_WRITE = config['internal_pilot_DB']['write_to_internal_pilot_DB']
SELF_REG_DEFAULT = config['internal_pilot_DB']['self_registration_default']

'''Live Tracking servers'''
FM_LIVE = config['flymaster_live_server']

'''Telegram Bot'''
TELEGRAM_API = secret.get('telegram', {}).get('API') or env.str('TELEGRAM_API')
TELEGRAM_CHANNEL = secret.get('telegram', {}).get('channel') or env.str('TELEGRAM_CHANNEL')

'''Ladders'''
LADDERS = config['ladders']