$title Modelo de programacion matematica Set Partition

$include data.inc

OPTION OPTCR=0.01;

VARIABLES
 X(j) si la ruta j se incluye en la solucion
 TC costo de viaje
 BINARY VARIABLE X
 
EQUATIONS
 TRAVEL costo de viaje
 INCLUDE(i) cada cliente se incluye en la solucion una unica vez;

 TRAVEL                          .. TC =E= SUM[j, R(j)*X(j)];
 INCLUDE(i)                      .. SUM[j, A(i,j)*X(j)] =G= 1

MODEL SPVRPSPD /all/;
SOLVE SPVRPSPD USING MIP MINIMIZING TC;
DISPLAY TC.L, X.L, INCLUDE.L;

FILE problem/ setpartition.dat/;
PUT problem;
PUT TC.L /;
LOOP(j, PUT j.tl, @8, X.L(j)/);