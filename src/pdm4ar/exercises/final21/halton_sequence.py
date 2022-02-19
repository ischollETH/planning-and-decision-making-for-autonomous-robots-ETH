# Based on: https://laszukdawid.com/2017/02/04/halton-sequence-in-python/

import numpy as np


def vdc_sequence(n, base=2):
    """The van der Corput sequence"""
    vdc, denom = 0, 1
    while n:
        denom *= base
        n, remainder = divmod(n, base)
        vdc += remainder / float(denom)
    return vdc


class HaltonSequence:
    def __init__(self, min_bounds, max_bounds, dimensions, size=10):
        self.primes = [2, 3, 5, 7, 11, 13]

        assert dimensions == len(min_bounds)
        assert dimensions == len(max_bounds)
        assert dimensions <= len(self.primes)

        self.dim = dimensions
        self.size = size
        self.seq = np.array(self.halton_sequence())
        self.seq = self.scale_to_size(min_bounds, max_bounds)
        self.samples_retrieved = 0
        self.min_bounds = min_bounds
        self.max_bounds = max_bounds
        #print("Halton Sequence newly initialized")

    def halton_sequence(self):
        """Computes the halton sequence based on the VdC-sequence"""
        seq = []
        for d in range(self.dim):
            base = self.primes[d]
            seq.append([vdc_sequence(i, base) for i in range(self.size)])
        return seq

    def scale_to_size(self, min_bounds, max_bounds):
        """Takes an array of minimum values and array of maximum values with size of the sample dimensionality"""
        bounds_diff = max_bounds - min_bounds
        assert min(bounds_diff) >= 0

        seq_max_scaled = self.seq * bounds_diff.repeat(self.size).reshape((self.dim, self.size))
        seq_min_max_scaled = seq_max_scaled + min_bounds.repeat(self.size).reshape((self.dim, self.size))

        return seq_min_max_scaled

    def get_next_sample(self):
        """Retrieves the next sample from the sequence"""
        sample = self.seq[:, self.samples_retrieved]
        self.samples_retrieved += 1

        if self.samples_retrieved >= self.size:
            self.size = self.size*2
            print(f'Halton sequence to small! - Increasing it to {self.size}')
            self.seq = np.array(self.halton_sequence())
            self.seq = self.scale_to_size(self.min_bounds, self.max_bounds)
            self.samples_retrieved = 0

        return sample
