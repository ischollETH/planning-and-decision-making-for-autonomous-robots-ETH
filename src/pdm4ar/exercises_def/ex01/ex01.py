from random import random
from typing import Tuple, List

import numpy as np
from reprep import Report
from zuper_commons.text import remove_escapes

from pdm4ar.exercises import ComparisonOutcome, compare_lexicographic
from pdm4ar.exercises_def import Exercise
from pdm4ar.exercises_def.ex01.ex01_sol import compare_lexicographic_sol


def wrap_exercise1(alg_in: List[Tuple[np.ndarray]]) -> List[ComparisonOutcome]:
    test_results = []
    for value in alg_in:
        test_results.append(compare_lexicographic(tuple(value[0]), tuple(value[1])))
    return test_results


def exercise1_report(algo_in: List[Tuple[np.ndarray]], algo_out: List[ComparisonOutcome]) -> Report:
    r = Report("Exercise1")
    correct_answers: int = 0
    for i, value in enumerate(algo_in):
        va, vb = tuple(value[0]), tuple(value[1])
        correct_output: ComparisonOutcome = compare_lexicographic_sol(va, vb)
        if correct_output == algo_out[i]:
            correct_answers += 1
        msg = f"Input:\na:\t{va}\tb:\t{vb}" f"\nOutput:\n\t{algo_out[i]}" f"\nExpectedOutput:\n\t{correct_output}"
        r.text(f"Test{i}", text=remove_escapes(msg))

    msg = f"You got {correct_answers}/{len(algo_in)} correct results!"
    r.text("ResultsInfo", text=remove_escapes(msg))
    return r


def get_exercise1() -> Exercise:
    test_values = []
    size = [2, 6]
    for i in range(40):
        if random() > 0.1:
            values = tuple(np.round(np.random.random(size) * 10))
        else:
            v0 = np.round(np.random.random(size[1]) * 10)
            values = (v0, v0)
        test_values.append(values)

    return Exercise[List[Tuple[np.ndarray]], List[ComparisonOutcome]](
        desc="This is exercise1 about lexicographic comparisons",
        algorithm=wrap_exercise1,
        report=exercise1_report,
        test_values=[
            test_values,
        ],
    )
