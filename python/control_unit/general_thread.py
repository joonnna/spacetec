import threading
import time

class GeneralThread(threading.Thread):
    def __init__(self, func, cleanup_func, timeout, args=None):
        threading.Thread.__init__(self)
        self._stop_event = threading.Event()
        self.func = func
        self.cleanup = cleanup_func
        self.timeout = timeout
        self._args = args
        self.daemon = True

    def run(self):
        while True:
            if self._should_stop():
                self.cleanup()
                print "EXITED THE FUCKING THREAD YOU DIPSHIT!"
                return

            if self._args == None:
                self.func()
            else:
                self.func(self._args)

            if self.timeout > 0.0:
                time.sleep(self.timeout)

    def stop(self):
        self._stop_event.set()
        print "Exiting thread"

    def _should_stop(self):
        return self._stop_event.is_set()


def new_thread(func, cleanup, timeout, args=None):
    thread = GeneralThread(func, cleanup, timeout, args)
    return thread
