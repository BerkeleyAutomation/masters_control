'''
Abtraction for a sequence of events
Author: Jacky Liang
'''
import logging
class Sequence:

    def __init__(self, data):
        self._data = data
        self._n = len(self._data)

    def subsampler(self, d, retain_features=False):
        if retain_features:
            data = [None for _ in range(0, self._n, d)]

            for i in range(len(data)):
                fts = self._data[i*d : (i+1)*d]
                recorded = False
                for ft in fts:
                    if ft is not None:
                        if recorded:
                            logging.warn("Aliasing occurred during Sequence subsampling operation!")
                            break
                        else:
                            data[i] = ft
                            recorded = True

            return Sequence(data)

        return Sequence([self._data[i] for i in range(0, self._n, d)])

    @property
    def data(self):
        return self._data
