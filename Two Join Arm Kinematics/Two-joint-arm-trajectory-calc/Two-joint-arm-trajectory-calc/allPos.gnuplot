set datafile separator ","
set xrange [-0.6:1.6]
set yrange [-0.6:1.6]
plot '01.csv' using 10:11 title "pos1" pt 7 ps 0.1, \
'02.csv' using 10:11 title "pos2" pt 7 ps 0.1, \
'20.csv' using 10:11 title "pos2N" pt 7 ps 0.1, \
'04.csv' using 10:11 title "pos4" pt 7 ps 0.1, \
'05.csv' using 10:11 title "pos5" pt 7 ps 0.1, \
'06.csv' using 10:11 title "pos6" pt 7 ps 0.1, \
'07.csv' using 10:11 title "pos7" pt 7 ps 0.1, \
'56.csv' using 10:11 title "pos56" pt 7 ps 0.1, \