set datafile separator ","
plot 'yeah.csv' using 1:8 title "x" pt 7 ps 0.1, \
'yeah.csv' using 1:9 title "y" pt 7 ps 0.1, \
'yeah.csv' using 1:10 title "xvel" pt 7 ps 0.1, \
'yeah.csv' using 1:11 title "yvel" pt 7 ps 0.1, \
'yeah.csv' using 1:12 title "xacc" pt 7 ps 0.1, \
'yeah.csv' using 1:13 title "yacc" pt 7 ps 0.1, \