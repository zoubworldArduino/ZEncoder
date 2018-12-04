# ZEncoder
Arduino  Lib for encoder of motor with ros.
It work like a PDEC peripheal.
just keep in mind that it is handler by interrupt and it cost in cpu time :
for 1 motor at 10k rpm with 1 tick per revolutions it cost 66kcyc per seconde (10000/60*450cyc) (without speed control)
and it can create latency about 10µs@48Mhz that could drop data on serial if serial interrupt happen to late.



