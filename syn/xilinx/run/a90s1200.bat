set name=a90s1200
rem set target=xc2v250-cs144-6
rem set target=xcv300e-pq240-8
set target=xc2s200-pq208-5

if "%2" == "" goto default
set target=%2
:default

cd ..\out

if "%1" == "" goto xst

copy ..\bin\%name%.pin %name%.ucf

ngdbuild -p %target% %1

goto builddone

:xst

xrom ROM1200 9 16 > ..\src\ROM1200_Echo.vhd
hex2rom ..\..\..\sw\echo100.hex rom1200 9b16u > rom1200_echo.ini
copy ..\out\rom1200_echo.ini + ..\bin\%name%.pin %name%.ucf

xst -ifn ../bin/%name%.scr -ofn ../log/%name%.srp
ngdbuild -p %target% %name%.ngc

:builddone

move %name%.bld ..\log

map -p %target% -cm speed -c 100 -tx on -o %name%_map %name%
move %name%_map.mrp ..\log

par -ol 2 -xe 0 -t 1 -c 0 %name%_map -w %name%
move %name%.par ..\log

trce %name%.ncd -o ../log/%name%.twr %name%_map.pcf

bitgen -w %name%
move %name%.bgn ..\log

cd ..\run
