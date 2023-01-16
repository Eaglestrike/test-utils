set datafile separator ","
plot 'yeah.csv' using 1:3 title "pos" pt 7 ps 0.1, \
'yeah.csv' using 1:5 title "vel" pt 7 ps 0.1, \
'yeah.csv' using 1:7 title "acc" pt 7 ps 0.1, \
'yeah.csv' using 1:15 title "test acc" pt 7 ps 0.1, \
'yeah.csv' using 1:17 title "pos2" pt 7 ps 0.1