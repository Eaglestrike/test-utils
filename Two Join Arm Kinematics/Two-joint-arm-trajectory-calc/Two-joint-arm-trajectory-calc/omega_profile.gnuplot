set datafile separator ","
plot 'yeah.csv' using 1:2 title "pos" pt 7 ps 0.1, \
'yeah.csv' using 1:4 title "vel" pt 7 ps 0.1, \
'yeah.csv' using 1:6 title "acc" pt 7 ps 0.1, \
'yeah.csv' using 1:14 title "test acc" pt 7 ps 0.1