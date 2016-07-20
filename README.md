# tailored-lwb
This is a contiki implementation of LWB and FS-LWB for sky motes.

The apps folder contains the LWB related codes. You can run the given simulation file using cooja. If you want to create your own simulation, ensure that you select "Radio medium" as "Multi-path Ray-tracer Medium (MRM)" instead of the default "UDGM".

By default, the nodes will be using LWB as communication mechanism with 10s interval. The forwarder selection can be enforced by defining "FORWARDER_SELECTION" as 1 in the "slot-def.h" file. The periodicity of LWB can also be changed by changing IPI parameter in this file.
