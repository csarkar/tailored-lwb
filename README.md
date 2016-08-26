# tailored-lwb
This is a contiki implementation of LWB and FS-LWB for sky motes.

The apps folder contains the LWB related codes. You can run the given simulation file using cooja. If you want to create your own simulation, ensure that you select "Radio medium" as "Multi-path Ray-tracer Medium (MRM)" instead of the default "UDGM".

By default, the nodes will be using LWB as communication mechanism with 10s interval. The forwarder selection can be enforced by defining "FORWARDER_SELECTION" as 1 in the "slot-def.h" file. The periodicity of LWB can also be changed by changing IPI parameter in this file.

Please cite the following article -

LWB and FS-LWB implementation for Sky platform using Contiki
https://arxiv.org/pdf/1607.06622.pdf

or just copy the bibtex content -

@misc{tailored_lwb,

  title={LWB and FS-LWB implementation for Sky nodes using Contiki},

  author={Sarkar, Chayan},

  howpublished = {arXiv preprint - \url{https://arxiv.org/pdf/1607.06622.pdf}},

  year={2016}

}

A big thanks to all the authors of the original works of LWB and FS-LWB.

LWB - http://dx.doi.org/10.1145/2426656.2426658

FS-LWB - http://dx.doi.org/10.1109/DCOSS.2013.73


Few suggestions about using the code -

(i) Though I have tested the old code (with contiki-2.4), it is not extensively tested on real nodes. So, better use the newer code and compile it with contiki-3.0.

(ii) In the project config file, keep COOJA=1 for both cooja-based testing and real nodes.

(iii) You can reduce the duty cycle of the nodes (a bit of idle listening for sync packet) by reducing the GLOSSY_DURATION and GLOSSY_SYNC_GUARD time. However, if you have nodes that has relatively high offset, then the nodes will be out of SYNC and the system breaks.

