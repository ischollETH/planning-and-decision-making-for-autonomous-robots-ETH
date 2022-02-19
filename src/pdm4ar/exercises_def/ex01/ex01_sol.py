from typing import Tuple

from pdm4ar.exercises import ComparisonOutcome, FIRST_PREFERRED, SECOND_PREFERRED, INDIFFERENT


def compare_lexicographic_sol(a: Tuple[float], b: Tuple[float]) -> ComparisonOutcome:
    """
    Sample solution
    """
    assert len(a) == len(b)

    for i in range(len(a)):
        if a[i] < b[i]:
            return FIRST_PREFERRED
        elif b[i] < a[i]:
            return SECOND_PREFERRED

    return INDIFFERENT
