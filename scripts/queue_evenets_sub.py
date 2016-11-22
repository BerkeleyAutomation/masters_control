'''
A wrapper class for queue events for subscription. Used for data recording.
Authro: Jacky Liang
'''
from multiprocessing import Queue

def QueueEventsSub:

    def __init__(self, maxsize=0):
        self._maxsize = maxsize
        self._q = Queue(maxsize=maxsize)

    def put_event(self, e):
        if self._q.qsize() == self._maxsize:
            self._q.get_nowait()

        self._q.put(e):

    def get_event(self):
        if self._q.qsize() == 0:
            return None
        return self._q.get()
