set datafile separator ","
plot 'test.csv' using 1:8 title "theta" pt 7 ps 0.1, \
'test.csv' using 1:9 title "phi" pt 7 ps 0.1