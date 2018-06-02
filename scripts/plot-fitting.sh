gnuplot -e  '
p "predict.log" u 0:3 every 5 w lp;
rep "predict.log" u 0:5 every 5 w lp;
pause -1;
'
gnuplot -e  '
p "predict.log" u 0:4 every 5 w lp;
rep "predict.log" u 0:6 every 5 w lp;
pause -1;
'
