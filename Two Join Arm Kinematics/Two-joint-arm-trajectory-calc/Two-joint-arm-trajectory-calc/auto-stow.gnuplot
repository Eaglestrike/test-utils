set datafile separator ","
set xrange [-2:2]
set yrange [-2:2]
plot '29.csv' using 10:11 title "pos" pt 7 ps 0.1, \
'92.csv' using 10:11 title "pos2" pt 7 ps 0.1, \