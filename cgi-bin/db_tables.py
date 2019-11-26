# coding: utf-8
from sqlalchemy import CHAR, Column, DECIMAL, Date, DateTime, Enum, Float, Index, String, TIMESTAMP, Table, Text, text, create_engine
from sqlalchemy.dialects.mysql import BIGINT, INTEGER, LONGTEXT, MEDIUMTEXT, TINYINT, VARCHAR
from sqlalchemy.ext.declarative import declarative_base
from myconn import Base, metadata

class CompResultView(Base):
    __table__ = Table( 'CompResultView', metadata,

    Column('comp_id', INTEGER(11), primary_key=True),
    Column('comp_name', String(100)),
    Column('comp_site', String(100)),
    Column('date_from', String(10)),
    Column('date_to', String(10)),
    Column('MD_name', String(100)),
    Column('contact', String(100)),
    Column('sanction', Enum('FAI 1', 'League', 'PWC', 'FAI 2', 'none'), server_default=text("'none'")),
    Column('type', Enum('RACE', 'Route', 'Team-RACE'), server_default=text("'RACE'")),
    Column('comp_code', String(8)),
    Column('restricted', Enum('open', 'registered'), server_default=text("'registered'")),
    Column('time_offset', Float(asdecimal=False)),
    Column('comp_class', Enum('PG', 'HG', 'mixed'), server_default=text("'PG'")),
    Column('website', String(100)),
    Column('formula', String(88)),
    Column('formula_class', Enum('gap', 'pwc', 'RTG'), server_default=text("'pwc'")),
    Column('team_size', INTEGER(11)),
    Column('overall_validity', Enum('ftv', 'all', 'round'), server_default=text("'ftv'")),
    Column('validity_param', Float(asdecimal=False), server_default=text("'0.75'")),
    Column('team_scoring', Enum('off', 'on'), server_default=text("'off'")),
    Column('team_over', INTEGER(2))
)

class CompetitionView(Base):
    __table__ = Table( 'CompetitionView', metadata,

    Column('comPk', INTEGER(11), primary_key=True),
    Column('comName', String(100)),
    Column('comLocation', String(100)),
    Column('comDateFrom', DateTime),
    Column('comDateTo', DateTime),
    Column('comMeetDirName', String(100)),
    Column('comContact', String(100)),
    Column('claPk', INTEGER(11), server_default=text("'0'")),
    Column('comSanction', Enum('FAI 1', 'League', 'PWC', 'FAI 2', 'none'), server_default=text("'none'")),
    Column('comType', Enum('RACE', 'Route', 'Team-RACE'), server_default=text("'RACE'")),
    Column('comCode', String(8)),
    Column('comEntryRestrict', Enum('open', 'registered'), server_default=text("'registered'")),
    Column('comTimeOffset', Float(asdecimal=False), server_default=text("'11'")),
    Column('comClass', Enum('PG', 'HG', 'mixed'), server_default=text("'PG'")),
    Column('comStyleSheet', String(128)),
    Column('comLocked', INTEGER(11), server_default=text("'0'")),
    Column('comExt', INTEGER(2), server_default=text("'0'")),
    Column('comExtUrl', String(100)),
    Column('forName', String(50)),
    Column('comOverallScore', Enum('ftv', 'all', 'round'), server_default=text("'ftv'")),
    Column('comOverallParam', Float(asdecimal=False), server_default=text("'0.75'")),
    Column('forNomGoal', Float(asdecimal=False), server_default=text("'0.3'")),
    Column('forMinDistance', Float(asdecimal=False), server_default=text("'5'")),
    Column('forNomDistance', Float(asdecimal=False), server_default=text("'45'")),
    Column('forNomTime', Float(asdecimal=False), server_default=text("'90'")),
    Column('forNomLaunch', Float(asdecimal=False), server_default=text("'0.96'")),
    Column('forScorebackTime', INTEGER(11), server_default=text("'5'")),
    Column('comTeamSize', INTEGER(11)),
    Column('comTeamScoring', Enum('off', 'on'), server_default=text("'off'")),
    Column('comTeamOver', INTEGER(2)),
    Column('forPk', BIGINT(11)),
    Column('forClass', String(3)),
    Column('forVersion', String(32)),
    Column('forComClass', String(5))
)

class CompObjectView(Base):
    __table__ = Table( 'CompObjectView', metadata,

    Column('comp_id', INTEGER(11), primary_key=True),
    Column('comp_name', String(100)),
    Column('comp_site', String(100)),
    Column('start_date', DateTime),
    Column('end_date', DateTime),
    Column('MD_name', String(100)),
    Column('contact', String(100)),
    Column('cat_id', INTEGER(11), server_default=text("'0'")),
    Column('sanction', Enum('FAI 1', 'League', 'PWC', 'FAI 2', 'none'), server_default=text("'none'")),
    Column('comp_type', Enum('RACE', 'Route', 'Team-RACE'), server_default=text("'RACE'")),
    Column('comp_code', String(8)),
    Column('restricted', Enum('open', 'registered'), server_default=text("'registered'")),
    Column('time_offset', Float(asdecimal=False), server_default=text("'11'")),
    Column('comp_class', Enum('PG', 'HG', 'mixed'), server_default=text("'PG'")),
    Column('stylesheet', String(128)),
    Column('locked', INTEGER(11), server_default=text("'0'")),
    Column('external', INTEGER(2), server_default=text("'0'")),
    Column('comp_url', String(100))
)

class PilotView(Base):
    __table__ = Table( 'PilotView', metadata,

    Column('pilPk', BIGINT(20), primary_key=True),
    Column('pilLogin', String(60)),
    Column('pilpass', String(255)),
    Column('pilEmail', String(100)),
    Column('pilFirstName', LONGTEXT),
    Column('pilLastName', LONGTEXT),
    Column('pilNat', LONGTEXT),
    Column('pilPhoneMobile', LONGTEXT),
    Column('pilSex', String(1)),
    Column('pilGliderBrand', LONGTEXT),
    Column('pilGlider', LONGTEXT),
    Column('gliGliderCert', LONGTEXT),
    Column('gliGliderClass', String(12)),
    Column('pilSponsor', LONGTEXT),
    Column('pilFAI', LONGTEXT),
    Column('pilCIVL', LONGTEXT),
    Column('pilLT24User', LONGTEXT),
    Column('pilATUser', LONGTEXT),
    Column('pilXContestUser', LONGTEXT)
)

class RegionWaypointView(Base):
    __table__ = Table( 'RegionWaypointView', metadata,

    Column('rwpPk', INTEGER(11), primary_key=True),
    Column('region_id', INTEGER(11)),
    Column('name', String(12)),
    Column('lat', Float(asdecimal=False)),
    Column('lon', Float(asdecimal=False)),
    Column('altitude', INTEGER(11)),
    Column('description', String(64))
)

class RegistrationView(Base):
    __table__ = Table( 'RegistrationView', metadata,

    Column('par_id', INTEGER(11), primary_key=True),
    Column('comp_id', INTEGER(11)),
    Column('pil_id', INTEGER(11)),
    Column('ID', INTEGER(5)),
    Column('name', String(50)),
    Column('birthdate', CHAR(10)),
    Column('sex', Enum('M', 'F')),
    Column('female', INTEGER(1), server_default=text("'0'")),
    Column('nat', CHAR(10)),
    Column('glider', String(100)),
    Column('glider_cert', String(20)),
    Column('sponsor', String(100)),
    Column('civl_id', INTEGER(10)),
    Column('fai_valid', TINYINT(1), server_default=text("'1'")),
    Column('fai_id', String(20)),
    Column('xcontest_id', String(20)),
    Column('team', String(100)),
    Column('paid', INTEGER(11), server_default=text("'0'"))
)

class RegisteredPilotView(Base):
    __table__ = Table( 'RegisteredPilotView', metadata,

    Column('par_id', INTEGER(11), primary_key=True),
    Column('comp_id', INTEGER(11)),
    Column('pil_id', INTEGER(11)),
    Column('ID', INTEGER(5)),
    Column('name', String(50)),
    Column('sex', String(1)),
    Column('nat', CHAR(10)),
    Column('glider', String(100)),
    Column('class', String(20)),
    Column('sponsor', String(100)),
    Column('civl', INTEGER(10)),
    Column('fai', String(20)),
    Column('team', String(100)),
)

class ResultView(Base):
    __table__ = Table( 'ResultView', metadata,

    Column('tarPk', INTEGER(11), primary_key=True),
    Column('parPk', INTEGER(11)),
    Column('tasPk', INTEGER(11)),
    Column('pilPk', INTEGER(11)),
    Column('pilName', LONGTEXT),
    Column('pilSponsor', LONGTEXT),
    Column('pilNationCode', String(10)),
    Column('pilSex', String(1)),
    Column('traGlider', String(100)),
    Column('traDHV', String(20)),
    Column('tarDistance', Float(asdecimal=False)),
    Column('tarSpeed', Float(asdecimal=False)),
    Column('tarStart', INTEGER(11)),
    Column('tarGoal', INTEGER(11)),
    Column('tarResultType', String(7)),
    Column('tarSS', INTEGER(11)),
    Column('tarES', INTEGER(11)),
    Column('tarTurnpoints', INTEGER(11)),
    Column('tarPenalty', Float(asdecimal=False)),
    Column('tarComment', Text),
    Column('tarPlace', INTEGER(11)),
    Column('tarSpeedScore', Float(asdecimal=False)),
    Column('tarDistanceScore', Float(asdecimal=False)),
    Column('tarArrivalScore', Float(asdecimal=False)),
    Column('tarDepartureScore', Float(asdecimal=False)),
    Column('tarScore', Float(asdecimal=False)),
    Column('tarLeadingCoeff', Float(asdecimal=False)),
    Column('tarFixedLC', Float(asdecimal=False)),
    Column('tarLastAltitude', INTEGER(11)),
    Column('tarLastTime', INTEGER(11))
)

class TaskFormulaView(Base):
    __table__ = Table( 'TaskFormulaView', metadata,

    Column('task_id', INTEGER(11), primary_key=True),
    Column('type', Enum('gap', 'pwc', 'RTG'), server_default=text("'pwc'")),
    Column('version', String(32)),
    Column('formula_name', String(50)),
    Column('arrival', Enum('off', 'on'), server_default=text("'on'")),
    Column('departure', Enum('off', 'on', 'leadout', 'kmbonus'), server_default=text("'on'")),
    Column('no_goal_penalty', Float(asdecimal=False), server_default=text("'1'")),
    Column('glide_bonus', Float(asdecimal=False), server_default=text("'4'")),
    Column('stopped_time_calc', Enum('atstopped', 'shortesttime'), server_default=text("'shortesttime'")),
    Column('arr_alt_bonus', Enum('off', 'on'), server_default=text("'off'")),
    Column('nominal_goal', Float(asdecimal=False), server_default=text("'0.3'")),
    Column('nominal_dist', Float(asdecimal=False), server_default=text("'0'")),
    Column('min_dist', Float(asdecimal=False), server_default=text("'0'")),
    Column('nominal_time', Float(asdecimal=False), server_default=text("'0'")),
    Column('nominal_launch', Float(asdecimal=False), server_default=text("'0.96'")),
    Column('score_back_time', BIGINT(13), server_default=text("'0'")),
    Column('tolerance', Float(asdecimal=False))
)

class TaskObjectView(Base):
    __table__ = Table( 'TaskObjectView', metadata,

    Column('task_id', INTEGER(11), primary_key=True),
    Column('comp_id', INTEGER(11)),
    Column('comp_code', String(8)),
    Column('comp_name', String(100)),
    Column('comp_site', String(100)),
    Column('time_offset', BIGINT(21)),
    Column('comp_class', Enum('PG', 'HG', 'mixed'), server_default=text("'PG'")),
    Column('date', String(10)),
    Column('task_name', String(100)),
    Column('reg_id', INTEGER(11)),
    Column('window_open_time', BIGINT(21)),
    Column('task_deadline', BIGINT(21)),
    Column('window_close_time', BIGINT(21)),
    Column('check_launch', Enum('on', 'off'), server_default=text("'off'")),
    Column('start_time', BIGINT(21)),
    Column('start_close_time', BIGINT(21)),
    Column('stopped_time', BIGINT(21)),
    Column('last_start_time', BIGINT(21)),
    Column('SS_interval', BIGINT(13)),
    Column('task_type', String(21)),
    Column('distance', Float(asdecimal=False)),
    Column('opt_dist', Float(asdecimal=False)),
    Column('opt_dist_to_SS', Float(asdecimal=False)),
    Column('opt_dist_to_ESS', Float(asdecimal=False)),
    Column('SS_distance', Float(asdecimal=False)),
    Column('comment', Text),
    Column('tasLocked', TINYINT(3), server_default=text("'0'")),
    Column('task_code', String(11)),
    Column('launch_valid', BIGINT(11))
)

class FlightResultView(Base):
    __table__ = Table( 'FlightResultView', metadata,

    Column('track_id', INTEGER(11), primary_key=True),
    Column('par_id', INTEGER(11)),
    Column('task_id', INTEGER(11)),
    Column('pil_id', INTEGER(11)),
    Column('name', LONGTEXT),
    Column('sponsor', LONGTEXT),
    Column('nat', String(10)),
    Column('sex', String(1)),
    Column('glider', String(100)),
    Column('class', String(20)),
    Column('distance_flown', Float(asdecimal=False)),
    Column('speed', Float(asdecimal=False)),
    Column('first_time', INTEGER(11)),
    Column('real_start_time', INTEGER(11)),
    Column('goal_time', INTEGER(11)),
    Column('last_time', INTEGER(11)),
    Column('result_type', String(7)),
    Column('SSS_time', INTEGER(11)),
    Column('ESS_time', INTEGER(11)),
    Column('best_waypoint_achieved', INTEGER(11)),
    Column('penalty', Float(asdecimal=False)),
    Column('comment', Text),
    Column('time_score', Float(asdecimal=False)),
    Column('distance_score', Float(asdecimal=False)),
    Column('arrival_score', Float(asdecimal=False)),
    Column('departure_score', Float(asdecimal=False)),
    Column('score', Float(asdecimal=False)),
    Column('lead_coeff', Float(asdecimal=False)),
    Column('fixed_LC', Float(asdecimal=False)),
    Column('ESS_altitude', INTEGER(11), server_default=text("'0'")),
    Column('goal_altitude', INTEGER(11)),
    Column('max_altitude', INTEGER(11), server_default=text("'0'")),
    Column('last_altitude', INTEGER(11)),
    Column('landing_altitude', INTEGER(11)),
    Column('landing_time', INTEGER(11)),
    Column('track_file', String(255)),
    Column('g_record', TINYINT(4))
)

class TaskResultView(Base):
    __table__ = Table( 'TaskResultView', metadata,

    Column('track_id', INTEGER(11), primary_key=True),
    Column('par_id', INTEGER(11)),
    Column('task_id', INTEGER(11)),
    Column('pil_id', INTEGER(11)),
    Column('name', LONGTEXT),
    Column('sponsor', LONGTEXT),
    Column('nat', String(10)),
    Column('sex', String(1)),
    Column('glider', String(100)),
    Column('class', String(20)),
    Column('distance', Float(asdecimal=False)),
    Column('speed', Float(asdecimal=False)),
    Column('first_time', INTEGER(11)),
    Column('real_start_time', INTEGER(11)),
    Column('goal_time', INTEGER(11)),
    Column('last_time', INTEGER(11)),
    Column('result', String(7)),
    Column('SS_time', INTEGER(11)),
    Column('ES_time', INTEGER(11)),
    Column('turnpoints_made', INTEGER(11)),
    Column('penalty', Float(asdecimal=False)),
    Column('comment', Text),
    Column('speed_points', Float(asdecimal=False)),
    Column('dist_points', Float(asdecimal=False)),
    Column('arr_points', Float(asdecimal=False)),
    Column('dep_points', Float(asdecimal=False)),
    Column('score', Float(asdecimal=False)),
    Column('lead_coeff', Float(asdecimal=False)),
    Column('fixed_LC', Float(asdecimal=False)),
    Column('ESS_altitude', INTEGER(11), server_default=text("'0'")),
    Column('goal_altitude', INTEGER(11)),
    Column('max_altitude', INTEGER(11), server_default=text("'0'")),
    Column('last_altitude', INTEGER(11)),
    Column('landing_altitude', INTEGER(11)),
    Column('landing_time', INTEGER(11)),
    Column('track_file', String(255)),
    Column('g_record', TINYINT(4))
)

class TaskStatsView(Base):
    __table__ = Table( 'TaskStatsView', metadata,

    Column('task_id', INTEGER(11), primary_key=True),
    Column('pilots_present', BIGINT(21)),
    Column('tot_dist_flown', Float(asdecimal=False)),
    Column('tot_dist_over_min', Float(asdecimal=False)),
    Column('pilots_launched', BIGINT(21)),
    Column('std_dev', Float(asdecimal=False)),
    Column('pilots_landed', BIGINT(21)),
    Column('pilots_goal', BIGINT(21)),
    Column('pilots_ess', BIGINT(21)),
    Column('max_distance', Float(asdecimal=False)),
    Column('min_dept_time', BIGINT(11), server_default=text("'0'")),
    Column('max_dept_time', BIGINT(11), server_default=text("'0'")),
    Column('first_SS', BIGINT(11), server_default=text("'0'")),
    Column('last_SS', BIGINT(11), server_default=text("'0'")),
    Column('min_ess_time', BIGINT(11), server_default=text("'0'")),
    Column('max_ess_time', BIGINT(11), server_default=text("'0'")),
    Column('fastest_in_goal', BIGINT(12), server_default=text("'0'")),
    Column('fastest', BIGINT(12), server_default=text("'0'")),
    Column('max_time', BIGINT(11), server_default=text("'0'"))
)

class TaskTotalsView(Base):
    __table__ = Table( 'TaskTotalsView', metadata,

    Column('tasPk', INTEGER(11), primary_key=True),
    Column('TotalPilots', BIGINT(21), server_default=text("'0'")),
    Column('TotalDistance', Float(asdecimal=False)),
    Column('TotDistOverMin', Float(asdecimal=False)),
    Column('TotalLaunched', DECIMAL(23, 0)),
    Column('Deviation', Float(asdecimal=False)),
    Column('TotalLanded', DECIMAL(23, 0)),
    Column('TotalGoal', DECIMAL(23, 0)),
    Column('TotalESS', DECIMAL(23, 0)),
    Column('maxDist', Float(asdecimal=False)),
    Column('firstStart', BIGINT(11), server_default=text("'0'")),
    Column('lastStart', BIGINT(11), server_default=text("'0'")),
    Column('firstSS', BIGINT(11), server_default=text("'0'")),
    Column('lastSS', BIGINT(11), server_default=text("'0'")),
    Column('firstESS', BIGINT(11), server_default=text("'0'")),
    Column('lastESS', BIGINT(11), server_default=text("'0'")),
    Column('minTimeGoal', BIGINT(12), server_default=text("'0'")),
    Column('minTime', BIGINT(12), server_default=text("'0'")),
    Column('lastTime', BIGINT(11), server_default=text("'0'")),
    Column('LCmin', Float(asdecimal=False), server_default=text("'0'"))
)

class TaskView(Base):
    __table__ = Table( 'TaskView', metadata,

    Column('tasPk', INTEGER(11), primary_key=True),
    Column('comPk', INTEGER(11)),
    Column('tasLastUpdate', TIMESTAMP, nullable=False,),
    Column('tasDate', Date),
    Column('tasName', String(100)),
    Column('regPk', INTEGER(11)),
    Column('tasTaskStart', DateTime),
    Column('tasFinishTime', DateTime),
    Column('tasLaunchClose', DateTime),
    Column('tasCheckLaunch', Enum('on', 'off'), server_default=text("'off'")),
    Column('tasStartTime', DateTime),
    Column('tasStartCloseTime', DateTime),
    Column('tasStoppedTime', DateTime),
    Column('tasLastStartTime', DateTime),
    Column('tasTaskType', Enum('race', 'elapsed time', 'free distance', 'distance with bearing'), server_default=text("'race'")),
    Column('tasDistance', Float(asdecimal=False)),
    Column('tasShortRouteDistance', Float(asdecimal=False)),
    Column('tasStartSSDistance', Float(asdecimal=False)),
    Column('tasEndSSDistance', Float(asdecimal=False)),
    Column('tasSSDistance', Float(asdecimal=False)),
    Column('tasSSInterval', INTEGER(11), server_default=text("'0'")),
    Column('tasQuality', Float(asdecimal=False)),
    Column('tasDistQuality', Float(asdecimal=False)),
    Column('tasTimeQuality', Float(asdecimal=False)),
    Column('tasLaunchQuality', Float(asdecimal=False)),
    Column('tasStopQuality', Float(asdecimal=False), server_default=text("'1'")),
    Column('tasAvailDistPoints', Float(asdecimal=False)),
    Column('tasAvailLeadPoints', Float(asdecimal=False)),
    Column('tasAvailTimePoints', Float(asdecimal=False)),
    Column('tasAvailArrPoints', Float(asdecimal=False)),
    Column('tasLaunchValid', INTEGER(11), server_default=text("'1'")),
    Column('forName', String(50)),
    Column('forClass', Enum('gap', 'pwc', 'RTG'), server_default=text("'pwc'")),
    Column('forVersion', String(32)),
    Column('forLinearDist', Float(asdecimal=False), server_default=text("'1'")),
    Column('forDiffDist', Float(asdecimal=False), server_default=text("'3'")),
    Column('forGoalSSpenalty', Float(asdecimal=False), server_default=text("'1'")),
    Column('forStoppedGlideBonus', Float(asdecimal=False), server_default=text("'4'")),
    Column('forStoppedElapsedCalc', Enum('atstopped', 'shortesttime'), server_default=text("'shortesttime'")),
    Column('comOverallScore', Enum('ftv', 'all', 'round'), server_default=text("'ftv'")),
    Column('comOverallParam', Float(asdecimal=False), server_default=text("'0.75'")),
    Column('forNomGoal', Float(asdecimal=False), server_default=text("'0.3'")),
    Column('forMinDistance', Float(asdecimal=False), server_default=text("'5'")),
    Column('forNomDistance', Float(asdecimal=False), server_default=text("'45'")),
    Column('forNomTime', Float(asdecimal=False), server_default=text("'90'")),
    Column('forNomLaunch', Float(asdecimal=False), server_default=text("'0.96'")),
    Column('forScorebackTime', INTEGER(11), server_default=text("'5'")),
    Column('tasDeparture', Enum('off', 'on', 'leadout', 'kmbonus'), server_default=text("'on'")),
    Column('tasArrival', Enum('off', 'on'), server_default=text("'on'")),
    Column('tasHeightBonus', Enum('off', 'on'), server_default=text("'off'")),
    Column('tasComment', Text),
    Column('tasLocked', TINYINT(3), server_default=text("'0'")),
    Column('tasCode', String(11)),
    Column('tasGoalAlt', Float(asdecimal=False)),
    Column('comCode', String(8)),
    Column('comName', String(100)),
    Column('comLocation', String(100)),
    Column('comDateFrom', DateTime),
    Column('comDateTo', DateTime),
    Column('comTimeOffset', Float(asdecimal=False), server_default=text("'11'")),
    Column('comClass', Enum('PG', 'HG', 'mixed'), server_default=text("'PG'")),
    Column('claPk', INTEGER(11), server_default=text("'0'")),
    Column('comEntryRestrict', Enum('open', 'registered'), server_default=text("'registered'")),
    Column('comExt', INTEGER(2), server_default=text("'0'")),
    Column('xccSiteID', INTEGER(11)),
    Column('xccToID', INTEGER(11)),
    Column('tasMargin', Float(asdecimal=False))
)


class TaskWaypointView(Base):
    __table__ = Table(  'TaskWaypointView', metadata,

    Column('id', INTEGER(11), primary_key=True),
    Column('rwpPk', INTEGER(11)),
    Column('task_id', INTEGER(11)),
    Column('n', INTEGER(11)),
    Column('how', Enum('entry', 'exit'), server_default=text("'entry'")),
    Column('radius', BIGINT(11)),
    Column('shape', Enum('circle', 'semicircle', 'line'), server_default=text("'circle'")),
    Column('type', Enum('waypoint', 'launch', 'speed', 'endspeed', 'goal'), server_default=text("'waypoint'")),
    Column('ssr_lat', Float(asdecimal=False)),
    Column('ssr_lon', Float(asdecimal=False)),
    Column('partial_distance', Float(asdecimal=False)),
    Column('lat', Float(asdecimal=False)),
    Column('lon', Float(asdecimal=False)),
    Column('name', String(12)),
    Column('description', String(64)),
    Column('altitude', BIGINT(21))
)

class TaskXContestWptView(Base):
    __table__ = Table( 'TaskXContestWptView', metadata,

    Column('tasPk',INTEGER(11), primary_key=True),
    Column('tasDate', Date),
    Column('tasGoalAlt', Float(asdecimal=False)),
    Column('xccSiteID', BIGINT(11)),
    Column('xccToID', BIGINT(11))
)

class UserView(Base):
    __table__ = Table( 'UserView', metadata,

    Column('usePk', BIGINT(20), primary_key=True),
    Column('useName', String(250)),
    Column('useLogin', String(60)),
    Column('useEmail', String(100))
)

class TrackFileView(Base):
    __table__ = Table( 'TrackFileView', metadata,

    Column('track_id', INTEGER(11), primary_key=True),
    Column('task_id', INTEGER(11)),
    Column('pil_id', INTEGER(11)),
    Column('filename', String(255)),
    Column('g_record', TINYINT(4))
)

schema_version = Table(
    'schema_version', metadata,
    Column('svKey', INTEGER(11), nullable=False, server_default=text("'0'")),
    Column('svWhen', TIMESTAMP, nullable=False, server_default=text("CURRENT_TIMESTAMP")),
    Column('svExtra', String(256))
)

class tblAirspace(Base):
    __tablename__ = 'tblAirspace'

    airPk = Column(INTEGER(11), primary_key=True)
    airName = Column(String(32))
    airClass = Column(Enum('G', 'C', 'D', 'E', 'X', 'R', 'P', 'Q', 'W', 'GP', 'CTR'), server_default=text("'C'"))
    airBase = Column(INTEGER(11))
    airTops = Column(INTEGER(11))
    airShape = Column(Enum('circle', 'wedge', 'polygon'), server_default=text("'circle'"))
    airCentreWP = Column(INTEGER(11))
    airRadius = Column(Float)

class tblAirspaceRegion(Base):
    __tablename__ = 'tblAirspaceRegion'

    argPk = Column(INTEGER(11), primary_key=True)
    argRegion = Column(String(32), nullable=False)
    argLatDecimal = Column(Float(asdecimal=False), nullable=False)
    argLongDecimal = Column(Float(asdecimal=False), nullable=False)
    argSize = Column(Float, nullable=False)

class tblAirspaceWaypoint(Base):
    __tablename__ = 'tblAirspaceWaypoint'

    awpPk = Column(INTEGER(11), primary_key=True)
    airPk = Column(INTEGER(11), nullable=False)
    airOrder = Column(INTEGER(11), nullable=False)
    awpConnect = Column(Enum('line', 'arc+', 'arc-'), server_default=text("'line'"))
    awpLatDecimal = Column(Float(asdecimal=False), nullable=False)
    awpLongDecimal = Column(Float(asdecimal=False), nullable=False)
    awpAngleStart = Column(Float)
    awpAngleEnd = Column(Float)
    awpRadius = Column(Float)

class tblCertification(Base):
    __tablename__ = 'tblCertification'

    cerPk = Column(INTEGER(11), primary_key=True)
    cerName = Column(String(15), nullable=False)
    comClass = Column(Enum('PG', 'HG', 'mixed'), nullable=False, server_default=text("'PG'"))

class tblClasCertRank(Base):
    __tablename__ = 'tblClasCertRank'

    claPk = Column(INTEGER(11), primary_key=True)
    cerPk = Column(INTEGER(11), nullable=False)
    ranPk = Column(INTEGER(11), nullable=False)

class tblClassification(Base):
    __tablename__ = 'tblClassification'

    claPk = Column(INTEGER(11), primary_key=True)
    claName = Column(String(60), nullable=False)
    comClass = Column(Enum('PG', 'HG', 'mixed'), nullable=False, server_default=text("'PG'"))
    claFem = Column(TINYINT(1), nullable=False, server_default=text("'1'"))
    claTeam = Column(TINYINT(1), nullable=False, server_default=text("'0'"))

class tblCompAuth(Base):
    __table__ = Table('tblCompAuth', metadata,

    Column('usePk', INTEGER(11), primary_key=True),
    Column('comPk', INTEGER(11)),
    Column('useLevel', Enum('read', 'write', 'admin'), server_default=text("'read'"))
)

class tblCompetition(Base):
    __tablename__ = 'tblCompetition'
    __table_args__ = (
        Index('comPk', 'comPk', 'comName', unique=True),
    )

    comPk = Column(INTEGER(11), primary_key=True)
    comName = Column(String(100), nullable=False)
    comLastUpdate = Column(TIMESTAMP, nullable=False, server_default=text("CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP"))
    comLocation = Column(String(100), nullable=False)
    comDateFrom = Column(DateTime, nullable=False)
    comDateTo = Column(DateTime, nullable=False)
    comMeetDirName = Column(String(100))
    comContact = Column(String(100))
    claPk = Column(INTEGER(11), nullable=False, server_default=text("'0'"))
    comSanction = Column(Enum('FAI 1', 'League', 'PWC', 'FAI 2', 'none'), nullable=False, server_default=text("'none'"))
    comType = Column(Enum('RACE', 'Route', 'Team-RACE'), server_default=text("'RACE'"))
    comCode = Column(String(8))
    comEntryRestrict = Column(Enum('open', 'registered'), server_default=text("'registered'"))
    comTimeOffset = Column(Float(asdecimal=False), server_default=text("'11'"))
    comClass = Column(Enum('PG', 'HG', 'mixed'), server_default=text("'PG'"))
    comStyleSheet = Column(String(128))
    comLocked = Column(INTEGER(11), server_default=text("'0'"))
    comExt = Column(INTEGER(2), nullable=False, server_default=text("'0'"))
    comExtUrl = Column(String(100))
    comPath  = Column(String(40))

class tblCountryCode(Base):
    __tablename__ = 'tblCountryCode'

    natName = Column(String(52), nullable=False)
    natIso2 = Column(String(2), nullable=False)
    natIso3 = Column(String(3), nullable=False)
    natId = Column(INTEGER(11), primary_key=True)
    natIso = Column(String(13))
    natRegion = Column(String(8))
    natSubRegion = Column(String(25))
    natRegionId = Column(INTEGER(11))
    natSubRegionId = Column(INTEGER(11))

class tblExtPilot(Base):
    __tablename__ = 'tblExtPilot'

    pilPk = Column(INTEGER(11), primary_key=True)
    pilFirstName = Column(String(100), index=True)
    pilLastName = Column(String(100), index=True)
    pilEmail = Column(VARCHAR(100), nullable=False, server_default=text("''"))
    pilNat = Column(String(255))
    pilPhoneMobile = Column(String(255))
    pilSex = Column(VARCHAR(1), nullable=False, server_default=text("''"))
    pilGliderBrand = Column(String(255))
    pilGlider = Column(String(255))
    gliGliderCert = Column(Enum('A', 'B', 'C', 'D', 'CCC', 'floater', 'kingpost', 'open', 'rigid'))
    gliGliderClass = Column(VARCHAR(12))
    pilSponsor = Column(String(255))
    pilFAI = Column(String(255), index=True)
    pilCIVL = Column(String(255))
    pilLT24User = Column(String(255))
    pilATUser = Column(String(255))
    pilXcontestUser = Column(String(255))

class tblExtResult(Base):
    __tablename__ = 'tblExtResult'

    etrPk = Column(INTEGER(11), primary_key=True)
    tasPk = Column(INTEGER(11), index=True)
    pilPk = Column(INTEGER(11))
    tarDistance = Column(Float(asdecimal=False))
    tarSpeed = Column(Float(asdecimal=False))
    tarStart = Column(INTEGER(11))
    tarGoal = Column(INTEGER(11))
    tarResultType = Column(Enum('abs', 'dnf', 'lo', 'goal', 'mindist'), server_default=text("'lo'"))
    tarSS = Column(INTEGER(11))
    tarES = Column(INTEGER(11))
    tarTurnpoints = Column(INTEGER(11))
    tarPenalty = Column(Float(asdecimal=False))
    tarComment = Column(Text)
    tarPlace = Column(INTEGER(11))
    tarSpeedScore = Column(Float(asdecimal=False))
    tarDistanceScore = Column(Float(asdecimal=False))
    tarArrivalScore = Column(Float(asdecimal=False))
    tarDepartureScore = Column(Float(asdecimal=False))
    tarScore = Column(Float(asdecimal=False))
    tarLeadingCoeff = Column(Float(asdecimal=False))
    tarFixedLC = Column(Float(asdecimal=False))
    tarESAltitude = Column(INTEGER(11), nullable=False, server_default=text("'0'"))
    tarMaxAltitude = Column(INTEGER(11), nullable=False, server_default=text("'0'"))
    tarLastAltitude = Column(INTEGER(11), server_default=text("'0'"))
    tarLastTime = Column(INTEGER(11))
    traGlider = Column(String(50))

class tblExtTask(Base):
    __tablename__ = 'tblExtTask'

    extPk = Column(INTEGER(11), primary_key=True)
    comPk = Column(INTEGER(11), nullable=False)
    comDateTo = Column(Date)
    tasName = Column(String(32))
    lcValue = Column(INTEGER(11), server_default=text("'450'"))
    tasQuality = Column(Float(asdecimal=False))
    tasTopScore = Column(INTEGER(11))
    extURL = Column(String(128))

class tblForComp(Base):
    __tablename__ = 'tblForComp'

    forPk = Column(INTEGER(11), index=True)
    comPk = Column(INTEGER(11), primary_key=True)
    forLastUpdate = Column(TIMESTAMP, nullable=False, server_default=text("CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP"))
    extForName = Column(String(50))
    comOverallScore = Column(Enum('ftv', 'all', 'round'), nullable=False, server_default=text("'ftv'"))
    comOverallParam = Column(Float(asdecimal=False), nullable=False, server_default=text("'0.75'"))
    forNomGoal = Column(Float(asdecimal=False), nullable=False, server_default=text("'0.3'"))
    forMinDistance = Column(Float(asdecimal=False), nullable=False, server_default=text("'5'"))
    forNomDistance = Column(Float(asdecimal=False), nullable=False, server_default=text("'45'"))
    forNomTime = Column(Float(asdecimal=False), nullable=False, server_default=text("'90'"))
    forNomLaunch = Column(Float(asdecimal=False), nullable=False, server_default=text("'0.96'"))
    forMinTime = Column(INTEGER(11))
    forScorebackTime = Column(INTEGER(11), nullable=False, server_default=text("'5'"))
    comTeamSize = Column(INTEGER(11))
    comTeamScoring = Column(Enum('off', 'on'), nullable=False, server_default=text("'off'"))
    comTeamOver = Column(INTEGER(2))

class tblFormula(Base):
    __tablename__ = 'tblFormula'

    forPk = Column(INTEGER(11), primary_key=True)
    forClass = Column(Enum('gap', 'pwc', 'RTG'), nullable=False, server_default=text("'pwc'"))
    forVersion = Column(String(32))
    forComClass = Column(Enum('PG', 'HG', 'mixed'), nullable=False, server_default=text("'PG'"))
    forName = Column(String(32), nullable=False)
    forArrival = Column(Enum('none', 'place', 'timed'), server_default=text("'none'"))
    forDeparture = Column(Enum('none', 'departure', 'leadout'), server_default=text("'leadout'"))
    forLinearDist = Column(Float(asdecimal=False), server_default=text("'1'"))
    forDiffDist = Column(Float(asdecimal=False), server_default=text("'3'"))
    forDistMeasure = Column(Enum('average', 'median'), server_default=text("'average'"))
    forDiffRamp = Column(Enum('fixed', 'flexible'), server_default=text("'fixed'"))
    forDiffCalc = Column(Enum('all', 'lo'), server_default=text("'all'"))
    forGoalSSpenalty = Column(Float(asdecimal=False), server_default=text("'1'"))
    forStoppedGlideBonus = Column(Float(asdecimal=False), server_default=text("'4'"))
    forMargin = Column(Float(asdecimal=False), server_default=text("'0.5'"))
    forStoppedElapsedCalc = Column(Enum('atstopped', 'shortesttime'), server_default=text("'shortesttime'"))
    forHeightArrBonus = Column(Float(asdecimal=False), server_default=text("'0'"))
    forHeightArrLower = Column(INTEGER(11), server_default=text("'200'"))
    forHeightArrUpper = Column(INTEGER(11), server_default=text("'3000'"))
    forWeightStart = Column(Float(asdecimal=False), server_default=text("'0.125'"))
    forWeightArrival = Column(Float(asdecimal=False), server_default=text("'0.175'"))
    forWeightSpeed = Column(Float(asdecimal=False), server_default=text("'0.7'"))

class tblGlider(Base):
    __tablename__ = 'tblGlider'

    gliPk = Column(INTEGER(11), primary_key=True)
    gliName = Column(String(32))
    gliManufacturer = Column(String(32))
    gliClass = Column(Enum('PG', 'HG'), server_default=text("'PG'"))
    gliDHV = Column(Enum('1', '1/2', '2', '2/3', 'competition', 'floater', 'kingpost', 'open', 'rigid'), server_default=text("'competition'"))

class tblLadder(Base):
    __tablename__ = 'tblLadder'

    ladPk = Column(INTEGER(11), primary_key=True)
    ladName = Column(String(100), nullable=False)
    ladComClass = Column(Enum('PG', 'HG'), nullable=False, server_default=text("'PG'"))
    ladNationCode = Column(INTEGER(11), server_default=text("'380'"))
    ladStart = Column(Date)
    ladEnd = Column(Date)
    ladIncExternal = Column(INTEGER(11), server_default=text("'0'"))
    ladImageM = Column(String(128))
    ladImageF = Column(String(128))

tblLadderComp = Table(
    'tblLadderComp', metadata,
    Column('ladPk', INTEGER(11)),
    Column('comPk', INTEGER(11))
)

tblLadderSeason = Table(
    'tblLadderSeason', metadata,
    Column('ladPk', INTEGER(11), nullable=False),
    Column('seasonYear', INTEGER(11), nullable=False),
    Column('ladActive', TINYINT(1), server_default=text("'1'")),
    Column('claPk', INTEGER(11), nullable=False),
    Column('ladOverallScore', Enum('all', 'ftv', 'round'), nullable=False, server_default=text("'ftv'")),
    Column('ladOverallParam', Float(asdecimal=False), nullable=False)
)

class tblLaunchSite(Base):
    __tablename__ = 'tblLaunchSite'

    lauPk = Column(INTEGER(11), primary_key=True)
    lauLaunch = Column(String(32), nullable=False)
    lauRegion = Column(String(32), nullable=False)
    lauLatDecimal = Column(Float(asdecimal=False), nullable=False)
    lauLongDecimal = Column(Float(asdecimal=False), nullable=False)
    lauAltitude = Column(Float(asdecimal=False), nullable=False)

class tblRanking(Base):
    __tablename__ = 'tblRanking'

    ranPk = Column(INTEGER(11), primary_key=True)
    ranName = Column(String(40), nullable=False)
    comClass = Column(Enum('PG', 'HG', 'mixed'), nullable=False, server_default=text("'PG'"))

class tblRegion(Base):
    __tablename__ = 'tblRegion'

    regPk = Column(INTEGER(11), primary_key=True)
    regCentre = Column(INTEGER(11))
    regRadius = Column(Float(asdecimal=False))
    regDescription = Column(String(64), nullable=False)
    regWptFileName = Column(String(50), nullable=False)
    regOpenAirFile = Column(String(50), nullable=False)

tblRegionAuth = Table(
    'tblRegionAuth', metadata,
    Column('usePk', INTEGER(11)),
    Column('comPk', INTEGER(11)),
    Column('useLevel', Enum('read', 'write', 'admin'), server_default=text("'read'"))
)

class tblRegionWaypoint(Base):
    __tablename__ = 'tblRegionWaypoint'

    rwpPk = Column(INTEGER(11), primary_key=True)
    regPk = Column(INTEGER(11))
    rwpName = Column(String(12), nullable=False)
    rwpLatDecimal = Column(Float(asdecimal=False), nullable=False)
    rwpLongDecimal = Column(Float(asdecimal=False), nullable=False)
    rwpAltitude = Column(Float(asdecimal=False), nullable=False)
    rwpDescription = Column(String(64))
    rwpOld = Column(TINYINT(4), nullable=False, server_default=text("'0'"))
    xccSiteID = Column(INTEGER(11))
    xccToID = Column(INTEGER(11))

tblRegionXCSites = Table(
    'tblRegionXCSites', metadata,
    Column('regPk', INTEGER(11), nullable=False),
    Column('xccSiteID', INTEGER(11), nullable=False)
)

class tblRegistration(Base):
    __tablename__ = 'tblRegistration'
    __table_args__ = (
        Index('pilPk', 'pilPk', 'comPk'),
    )

    parPk = Column(INTEGER(11), primary_key=True)
    comPk = Column(INTEGER(11))
    pilPk = Column(INTEGER(11))
    regID = Column(INTEGER(5))
    regName = Column(String(50))
    regBirthdate = Column(CHAR(10))
    regSex = Column(Enum('M', 'F'), nullable=False, server_default=text("'M'"))
    regNat = Column(CHAR(10))
    regGlider = Column(String(100))
    regCert = Column(String(20))
    regClass = Column(String(50))
    regSponsor = Column(String(100))
    regCIVL = Column(INTEGER(10))
    regValidFAI = Column(TINYINT(1), nullable=False, server_default=text("'1'"))
    regFAI = Column(String(20))
    regXC = Column(String(20))
    regTeam = Column(String(100))
    regPaid = Column(INTEGER(11), server_default=text("'0'"))
    regHours = Column(INTEGER(11), server_default=text("'200'"))

class tblResultFile(Base):
    __tablename__ = 'tblResultFile'

    refPk = Column(INTEGER(11), primary_key=True)
    comPk = Column(INTEGER(11), nullable=False)
    tasPk = Column(INTEGER(11))
    refTimestamp = Column(INTEGER(11), nullable=False)
    refJSON = Column(MEDIUMTEXT)
    refStatus = Column(VARCHAR(255))
    refVisible = Column(TINYINT(1), nullable=False, server_default=text("'0'"))

class tblTask(Base):
    __tablename__ = 'tblTask'

    tasPk = Column(INTEGER(11), primary_key=True)
    comPk = Column(INTEGER(11), index=True)
    tasLastUpdate = Column(TIMESTAMP, nullable=False, server_default=text("CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP"))
    tasName = Column(String(100))
    tasDate = Column(Date, nullable=False)
    regPk = Column(INTEGER(11))
    tasTaskStart = Column(DateTime)
    tasFinishTime = Column(DateTime)
    tasLaunchClose = Column(DateTime)
    tasCheckLaunch = Column(Enum('on', 'off'), server_default=text("'off'"))
    tasStartTime = Column(DateTime)
    tasStartCloseTime = Column(DateTime)
    tasStoppedTime = Column(DateTime)
    tasLastStartTime = Column(DateTime)
    tasFastestTime = Column(INTEGER(11))
    tasFirstDepTime = Column(INTEGER(11))
    tasFirstArrTime = Column(INTEGER(11))
    tasMaxDistance = Column(Float(asdecimal=False))
    tasResultsType = Column(String(20))
    tasTaskType = Column(Enum('race', 'elapsed time', 'free distance', 'distance with bearing'), server_default=text("'race'"))
    tasDistance = Column(Float(asdecimal=False))
    tasShortRouteDistance = Column(Float(asdecimal=False))
    tasStartSSDistance = Column(Float(asdecimal=False))
    tasEndSSDistance = Column(Float(asdecimal=False))
    tasSSDistance = Column(Float(asdecimal=False))
    tasSSInterval = Column(INTEGER(11), server_default=text("'0'"))
    tasTotalDistanceFlown = Column(Float(asdecimal=False))
    tasTotDistOverMin = Column(Float(asdecimal=False))
    tasQuality = Column(Float(asdecimal=False))
    tasDistQuality = Column(Float(asdecimal=False))
    tasTimeQuality = Column(Float(asdecimal=False))
    tasLaunchQuality = Column(Float(asdecimal=False))
    tasStopQuality = Column(Float(asdecimal=False), server_default=text("'1'"))
    tasAvailDistPoints = Column(Float(asdecimal=False))
    tasAvailLeadPoints = Column(Float(asdecimal=False))
    tasAvailTimePoints = Column(Float(asdecimal=False))
    tasAvailArrPoints = Column(Float(asdecimal=False))
    tasLaunchValid = Column(INTEGER(11), server_default=text("'1'"))
    tasPilotsLaunched = Column(INTEGER(11))
    tasPilotsTotal = Column(INTEGER(11))
    tasPilotsES = Column(INTEGER(11))
    tasPilotsLO = Column(INTEGER(11))
    tasPilotsGoal = Column(INTEGER(11))
    tasDeparture = Column(Enum('off', 'on', 'leadout', 'kmbonus'), server_default=text("'on'"))
    tasArrival = Column(Enum('off', 'on'), server_default=text("'on'"))
    tasHeightBonus = Column(Enum('off', 'on'), server_default=text("'off'"))
    tasComment = Column(Text)
    tasLocked = Column(TINYINT(3), nullable=False, server_default=text("'0'"))
    tasMarginOverride = Column(Float(asdecimal=False))
    tasPath  = Column(String(40))

class tblTaskAirspace(Base):
    __tablename__ = 'tblTaskAirspace'

    taPk = Column(INTEGER(11), primary_key=True)
    tasPk = Column(INTEGER(11), nullable=False)
    airPk = Column(INTEGER(11), nullable=False)

class tblTaskResult(Base):
    __tablename__ = 'tblTaskResult'
    __table_args__ = (
        Index('tarPk', 'tarPk', 'tasPk', 'traPk', unique=True),
    )

    tarPk = Column(INTEGER(11), primary_key=True)
    pilPk = Column(INTEGER(11), nullable=False)
    tasPk = Column(INTEGER(11), index=True)
    parPk = Column(INTEGER(11), index=True)
    tarLastUpdate = Column(TIMESTAMP, nullable=False, server_default=text("CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP"))
    traFile = Column(String(255))
    traGRecordOk = Column(TINYINT(4), server_default=text("'1'"))
    tarDistance = Column(Float(asdecimal=False))
    tarLaunch = Column(INTEGER(11))
    tarStart = Column(INTEGER(11))
    tarSS = Column(INTEGER(11))
    tarES = Column(INTEGER(11))
    tarGoal = Column(INTEGER(11))
    tarLastTime = Column(INTEGER(11))
    tarSpeed = Column(Float(asdecimal=False))
    tarTurnpoints = Column(INTEGER(11))
    tarESAltitude = Column(INTEGER(11), nullable=False, server_default=text("'0'"))
    tarGoalAltitude = Column(INTEGER(11), nullable=False, server_default=text("'0'"))
    tarMaxAltitude = Column(INTEGER(11), nullable=False, server_default=text("'0'"))
    tarLastAltitude = Column(INTEGER(11), server_default=text("'0'"))
    tarLandingTime = Column(INTEGER(11), nullable=False, server_default=text("'0'"))
    tarLandingAltitude = Column(INTEGER(11), nullable=False, server_default=text("'0'"))
    tarResultType = Column(Enum('abs', 'dnf', 'lo', 'goal', 'mindist'), server_default=text("'lo'"))
    tarPenalty = Column(Float(asdecimal=False))
    tarComment = Column(Text)
    tarPlace = Column(INTEGER(11))
    tarDistanceScore = Column(Float(asdecimal=False))
    tarSpeedScore = Column(Float(asdecimal=False))
    tarArrivalScore = Column(Float(asdecimal=False))
    tarDepartureScore = Column(Float(asdecimal=False))
    tarScore = Column(Float(asdecimal=False))
    tarLeadingCoeff = Column(Float(asdecimal=False))
    tarFixedLC = Column(Float(asdecimal=False))

class tblTaskWaypoint(Base):
    __tablename__ = 'tblTaskWaypoint'

    tawPk = Column(INTEGER(11), primary_key=True)
    tasPk = Column(INTEGER(11), index=True)
    rwpPk = Column(INTEGER(11), index=True)
    tawNumber = Column(INTEGER(11), nullable=False)
    tawTime = Column(INTEGER(11))
    tawType = Column(Enum('waypoint', 'launch', 'speed', 'endspeed', 'goal'), index=True, server_default=text("'waypoint'"))
    tawHow = Column(Enum('entry', 'exit'), server_default=text("'entry'"))
    tawShape = Column(Enum('circle', 'semicircle', 'line'), server_default=text("'circle'"))
    tawAngle = Column(INTEGER(11))
    tawRadius = Column(INTEGER(11))
    ssrLatDecimal = Column(Float(asdecimal=False))
    ssrLongDecimal = Column(Float(asdecimal=False))
    ssrCumulativeDist = Column(Float(asdecimal=False))

class tblTeam(Base):
    __tablename__ = 'tblTeam'

    teaPk = Column(INTEGER(11), primary_key=True)
    comPk = Column(INTEGER(11), index=True)
    teaName = Column(String(64))
    teaScoring = Column(INTEGER(11))

class tblTeamPilot(Base):
    __tablename__ = 'tblTeamPilot'
    __table_args__ = (
        Index('indTeamPilot', 'teaPk', 'pilPk'),
    )

    tepPk = Column(INTEGER(11), primary_key=True)
    teaPk = Column(INTEGER(11))
    pilPk = Column(INTEGER(11))
    tepPreference = Column(INTEGER(11), nullable=False, server_default=text("'1'"))
    tepModifier = Column(Float)

class tblTeamResult(Base):
    __tablename__ = 'tblTeamResult'

    terPk = Column(INTEGER(11), primary_key=True)
    tasPk = Column(INTEGER(11))
    traPk = Column(INTEGER(11))
    terDistance = Column(Float(asdecimal=False))
    terSpeed = Column(Float(asdecimal=False))
    terStart = Column(INTEGER(11))
    terGoal = Column(INTEGER(11))
    terResultType = Column(Enum('abs', 'dnf', 'lo', 'goal'), server_default=text("'lo'"))
    terSS = Column(INTEGER(11))
    terES = Column(INTEGER(11))
    terTurnpoints = Column(INTEGER(11))
    terPenalty = Column(Float(asdecimal=False))
    terComment = Column(Text)
    terPlace = Column(INTEGER(11))
    terSpeedScore = Column(Float(asdecimal=False))
    terDistanceScore = Column(Float(asdecimal=False))
    terArrival = Column(Float(asdecimal=False))
    terDeparture = Column(Float(asdecimal=False))
    terScore = Column(Float(asdecimal=False))
    terLeadingCoeff = Column(Float(asdecimal=False))

class tblTeamTask(Base):
    __tablename__ = 'tblTeamTask'

    tetPk = Column(INTEGER(11), primary_key=True)
    tasPk = Column(INTEGER(11))
    teaPk = Column(INTEGER(11))
    tetStartGate = Column(DateTime)

class tblTrack(Base):
    __tablename__ = 'tblTrack'

    traPk = Column(INTEGER(11), primary_key=True)
    pilPk = Column(INTEGER(11), nullable=False, index=True)
    witnessPk = Column(INTEGER(11))
    traClass = Column(Enum('PG', 'HG'), server_default=text("'PG'"))
    traGlider = Column(String(32))
    traDHV = Column(Enum('A', 'B', 'C', 'D', 'CCC', 'floater', 'kingpost', 'open', 'rigid'), server_default=text("'CCC'"))
    traDate = Column(Date, nullable=False)
    traStart = Column(DateTime)
    traArea = Column(Float(asdecimal=False))
    traLength = Column(Float(asdecimal=False))
    traScore = Column(Float(asdecimal=False))
    traSafety = Column(Enum('safe', 'maybe', 'unsafe'), server_default=text("'safe'"))
    traConditions = Column(INTEGER(11), server_default=text("'5'"))
    traOriginal = Column(String(128))
    traDuration = Column(INTEGER(11), server_default=text("'0'"))
    traGRecordOk = Column(INTEGER(11), server_default=text("'0'"))
    traInAir = Column(INTEGER(11), server_default=text("'0'"))
    traFile = Column(String(255))

tblUserSession = Table(
    'tblUserSession', metadata,
    Column('usePk', INTEGER(11), nullable=False),
    Column('useSession', String(128)),
    Column('useIP', String(32)),
    Column('useSessTime', TIMESTAMP, nullable=False, server_default=text("CURRENT_TIMESTAMP")),
    Column('useLastTime', TIMESTAMP, nullable=False, server_default=text("'0000-00-00 00:00:00'"))
)

class tblXContestCode(Base):
    __tablename__ = 'tblXContestCodes'

    xccSiteID = Column(INTEGER(11))
    xccSiteName = Column(String(40))
    xccToID = Column(INTEGER(11), primary_key=True)
    xccToName = Column(String(40), nullable=False)
    xccAlt = Column(INTEGER(11))
    xccISO = Column(String(2), nullable=False)
    xccCountryName = Column(String(42))
