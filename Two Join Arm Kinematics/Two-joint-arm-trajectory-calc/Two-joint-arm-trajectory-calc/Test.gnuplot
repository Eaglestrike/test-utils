set datafile separator ","
set xrange [-0.4:2]
set yrange [-0.4:2]
plot 'test.csv' using 10:11 title "pos" pt 7 ps 0.1, \