'''
Abtraction for a pose or joint trajectory
Author: Jacky Liang
'''

class Trajectory:

    def __init__(self, times, data):
        self._times = times
        self._data = data
        self._n = len(self._data)

    def subsampler(self, d):
        return [self._data[i] for i in range(0, self._n, d)]
