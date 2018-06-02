gnuplot -e  '
set title "pitch";
p "../log/diabolo_system/simulate.log" u 0:3 w lp;
pause -1;
'
gnuplot -e  '
set title "yaw";
p "../log/diabolo_system/simulate.log" u 0:4 w lp;
pause -1;
'
