cd ..\out

hex2rom ..\..\..\sw\sine2313.hex ROM2313 10b16s > ..\src\ROM2313_Sine_leo.vhd

spectrum -file ..\bin\a90s2313.tcl
move exemplar.log ..\log

cd ..\run

a90s2313 a90s2313.edf xc2s200-pq208-5
