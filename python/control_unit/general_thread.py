import threading
import time

class GeneralThread(threading.Thread):
    def __init__(self, func, cleanup_func, timeout, args=None):
        threading.Thread.__init__(self)
        self._stop_event = threading.Event()
        self._func = func
        self._cleanup = cleanup_func
        self._timeout = timeout
        self._args = args

    def run(self):
        while True:
            if self._should_stop():
                self._cleanup()
                return

            if self._args == None:
                self._func()
            else:
                self._func(self._args)

            if self._timeout > 0.0:
                time.sleep(self._timeout)

    def stop(self):
        self._stop_event.set()
        print "Exiting thread"

    def _should_stop(self):
        return self._stop_event.is_set()


def new_thread(func, cleanup, timeout, args=None):
    thread = GeneralThread(func, cleanup, timeout, args)
    return thread
