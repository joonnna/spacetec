from client import *

client = Udpclient()

data = "/home/machinekit/machinekit/spacetec/data_files/ptudata"

client.run(data)


Traceback (most recent call last):
File "/usr/lib/python2.7/threading.py", line 810, in __bootstrap_inner
    self.run()
File "/usr/lib/python2.7/threading.py", line 763, in run
    self.__target(*self.__args, **self.__kwargs)
    File "/usr/local/lib/python2.7/dist-packages/pymachinetalk/machinetalk_core/halremote/halrcompsubscribe.py", line 180, in _socket_worker
self._socket_message_received(socket)
    File "/usr/local/lib/python2.7/dist-packages/pymachinetalk/machinetalk_core/halremote/halrcompsubscribe.py", line 258, in _socket_message_received
self._fsm.any_msg_received()
    File "/usr/local/lib/python2.7/dist-packages/fysom/__init__.py", line 303, in fn
    self.transition()
File "/usr/local/lib/python2.7/dist-packages/fysom/__init__.py", line 296, in _tran
    self._enter_state(e)
File "/usr/local/lib/python2.7/dist-packages/fysom/__init__.py", line 345, in _enter_state
    for fnname in ['onenter' + e.dst, 'on' + e.dst]:
    TypeError: cannot concatenate 'str' and 'bool' objects

