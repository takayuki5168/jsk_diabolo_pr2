gnuplot -e  '
p "../log/fitting_pitch.log" u 0:1 w lp;
rep "../log/fitting_pitch.log" u 0:2 w lp;
pause -1;
'
gnuplot -e  '
p "../log/fitting_yaw.log" u 0:1 w lp;
rep "../log/fitting_yaw.log" u 0:2 w lp;
pause -1;
'
