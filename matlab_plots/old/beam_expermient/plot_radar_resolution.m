close all;clear;clc;

datalog = readtable("6843_data_log.csv");

plot(datalog.Var2, '.')
figure 

plot(datalog.Var3, '.')
figure 

plot(datalog.Var4, '.')
figure 

plot(datalog.Var5, '.')
figure 

plot(datalog.Var6, '.')
figure 

plot(datalog.Var7, '.')