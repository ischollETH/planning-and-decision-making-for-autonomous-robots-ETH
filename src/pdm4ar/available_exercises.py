from typing import Mapping

from frozendict import frozendict

from pdm4ar.exercises_def import *
from pdm4ar.exercises_def.structures import Exercise

available_exercises: Mapping[str, Callable[[], Exercise]] = frozendict(
    {
        "test": get_test_exercise,
        "exercise1": get_exercise1,
        "exercise2": get_exercise2,
        "exercise3": get_exercise3,
        "exercise4": get_exercise4,
        "final21": get_final21,
    }
)
