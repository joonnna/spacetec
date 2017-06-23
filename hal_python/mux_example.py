from machinekit import hal
from machinekit import rtapi as rt

rt.newthread('main-thread', 1000000, fp=True)
input0 = hal.newsig('input0', hal.HAL_FLOAT)
input1 = hal.newsig('input1', hal.HAL_FLOAT)
output = hal.newsig('output', hal.HAL_FLOAT)

mux = rt.newinst("muxn", "muxn")
mux.pin("in0").link(input0)
mux.pin("in1").link(input1)
mux.pin("out").link(output)
hal.addf(mux.name, "main-thread")

rcomp = hal.RemoteComponent("muxdemo", timer=100)
rcomp.newpin("button0", hal.HAL_FLOAT, hal.HAL_OUT)
rcomp.newpin("button1", hal.HAL_FLOAT, hal.HAL_OUT)
rcomp.newpin("led", hal.HAL_FLOAT, hal.HAL_IN)
rcomp.ready()

rcomp.pin('button0').link(input0)
rcomp.pin('button1').link(input1)
rcomp.pin('led').link(output)

hal.start_threads()

hal.loadusr('haltalk')
