from dataclasses import dataclass
from calcUtils import c_round

# integrate_precision is used to determine how many speed section slices will be created to calculate LC
# slices = 10 ^ integrate_precision
integrate_precision = 4

# weight matrix calculation
@dataclass(frozen=True)
class toDoWeight:
    ratio: float
    rising: float
    falling: float

def weight_matrix(precision: int = integrate_precision) -> list:
    matrix = []
    slices = 10**precision
    for i in range(slices):
        ratio = i/slices
        matrix.append(toDoWeight(ratio, weightRising(ratio), weightFalling(ratio)))
    return matrix


def lc_calculation(lc, result, fix, next_fix) -> float:
    """ Lead Coefficient formula from GAP2020
        11.3.1 Leading coefficient
        Each started pilot’s track log is used to calculate the leading coefficient (LC),
        by calculating the area underneath a graph defined by each track point’s time,
        and the distance to ESS at that time. The times used for this calculation are given in seconds
        from the moment when the first pilot crossed SSS, to the time when the last pilot reached ESS.
        For pilots who land out after the last pilot reached ESS, the calculation keeps going until they land.
        The distances used for the LC calculation are given in kilometers and are the distance from each
        point’s position to ESS, starting from SSS, but never more than any previously reached distance.
        This means that the graph never “goes back”: even if the pilot flies away from goal for a while,
        the corresponding points in the graph will use the previously reached best distance towards ESS.
    """

    progress = lc.best_dist_to_ess[0] - lc.best_dist_to_ess[1]
    if progress <= 0:
        return 0

    time = next_fix.rawtime - lc.best_start_time
    weight = weight_calc(toDo(lc.best_dist_to_ess[1], lc.ss_distance))
    return 0 if weight == 0 else weight * progress * time


def tot_lc_calculation(res, t) -> float:
    """Function to calculate final Leading Coefficient for pilots,
    that needs to be done when all tracks have been scored"""

    if res.result_type in ('abs', 'dnf', 'mindist', 'nyp') or not res.SSS_time:
        '''pilot did't make Start or has no track'''
        return 0

    ss_distance = t.SS_distance / 1000  # in Km
    if res.ESS_time:
        '''nothing to do'''
        landed_out = 0
    else:
        '''pilot did not make ESS'''
        best_dist_to_ess = max(0, res.best_dist_to_ESS / 1000)  # in Km
        missing_time = t.max_time - t.start_time
        landed_out = missing_area(missing_time, best_dist_to_ess, ss_distance)
    return (res.fixed_LC + landed_out) / (1800 * ss_distance)


def missing_area(time_interval: float, best_distance_to_ESS: float, ss_distance: float) -> float:
    """calculates medium weight for missing portion, missing area using mean weight value"""
    return weightFalling(toDo(best_distance_to_ESS, ss_distance)) * time_interval * best_distance_to_ESS


def toDo(dist_to_ess: float, ss_distance: float) -> float:
    return dist_to_ess / ss_distance


def weightRising(p: float) -> float:
    return (1 - 10 ** (9 * p - 9)) ** 5


def weightFalling(p: float) -> float:
    return (1 - 10 ** (-3 * p)) ** 2


def weight_calc(p: float) -> float:
    return weightRising(p) * weightFalling(p)


# Integrated calculation
def lc_calculation_integrate(lc, result, fix, next_fix) -> float:
    """ Lead Coefficient formula from PWC2023
        A1.1.1 Leading coefficient (LC)
        Each started pilot’s track log is used to calculate the Leading Coefficient (LC), 
        by calculating the area underneath a graph defined by each track point’s time, 
        and the distance to ESS at that time. 
        The times used for this calculation are given in seconds from the moment when the first pilot crossed SSS, 
        to the time when the last pilot reached ESS. For pilots who land out after the last pilot reached ESS, 
        the calculation keeps going until they land. 
        The distances used for the LC calculation are given in Kilometres and are the distance from e
        ach point’s position to ESS, starting from SSS, but never more than any previously reached distance. 
        This means that the graph never “goes back”: even if the pilot flies away from goal for a while, 
        the corresponding points in the graph will use the previously reached best distance towards ESS. 
        Important: Previous versions of the formula used distances to ESS squared to increase the number 
        of Leading Points awarded for leading out early in the task. This version uses a more complex 
        weighting according to distance from ESS to give no leading points at the start, 
        rising rapidly afterwards to give a flat section after about 20% of the speed section and, 
        finally, a similar linear function of distance from ESS after about 60% of the speed section.

        Airscore creates an array of tuples (ratio, weightRising, weightFalling) at fixed best_dist_to_ess / ss_distance
    """

    if lc.best_dist_to_ess[0] - lc.best_dist_to_ess[1] <= 0:
        return 0

    ratio = c_round(toDo(lc.best_dist_to_ess[1], lc.ss_distance), integrate_precision)
    index = int(ratio * lc.slices)
    if lc.latest_index and index >= lc.latest_index:
        return 0

    time = next_fix.rawtime - lc.best_start_time
    result = sum(weight * lc.slice_dist * time for weight in weight_calc_integrate(lc.matrix, lc.latest_index, index))
    lc.latest_index = index
    return result


def missing_area_integrate(matrix: list, id0: int, idx: int) -> list:
    result = [matrix[i].falling for i in range(id0, idx, -1)]
    return result


def weight_calc_integrate(matrix: list, id0: int, idx: int) -> list:
    result = [matrix[i].rising * matrix[i].falling for i in range(id0, idx, -1)]
    return result


def tot_lc_calculation_integrate(res, t) -> float:
    """Function to calculate final Leading Coefficient for pilots,
    that needs to be done when all tracks have been scored
    This version integrates weight value on fixed slices """

    if res.result_type in ('abs', 'dnf', 'mindist', 'nyp') or not res.SSS_time:
        '''pilot did't make Start or has no track'''
        return 0

    ss_distance = t.SS_distance / 1000  # in Km
    landed_out = 0
    if not res.ESS_time:
        '''pilot did not make ESS'''
        best_dist_to_ess = max(0, res.best_dist_to_ESS / 1000)  # in Km
        if best_dist_to_ess > 0:
            ratio = c_round(toDo(best_dist_to_ess, ss_distance), integrate_precision)
            index = min(len(t.formula.matrix) - 1, int(ratio * len(t.formula.matrix)))  # avoid out of range if pilot bombed out at Start
            missing_time = t.max_time - t.start_time
            landed_out = sum(weight * t.formula.slice_dist * missing_time for weight in missing_area_integrate(t.formula.matrix, index, 0))
    return (res.fixed_LC + landed_out) / (1800 * ss_distance)
