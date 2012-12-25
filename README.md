Infrared Temperature Controller using the attiny85
==================================================

Using your logic analyzer, record the signals from the remote, then export the file into a csv file that
would contain the timings & signal.  The power_button.csv is an example output from my Saleae logic analyzer.

Run the lasko_ir.rb to analyze the file, the output will be used as the IR code signal array.  See the 
power_button variable in main.cpp as an example.
