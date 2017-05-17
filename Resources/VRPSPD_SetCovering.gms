$title Modelo de Cubrimiento para el VRPSPD

OPTION OPTCR=0.001;

$include data.inc

VARIABLES
 X(j) si la ruta j se incluye en la solucion
 TC costo de viaje
 POSITIVE VARIABLE X;

EQUATIONS
 TRAVEL costo de viaje
 INCLUDE(i) cada cliente se incluye en la solucion una unica vez;

 TRAVEL                          .. TC =E= SUM[j, R(j)*X(j)];
 INCLUDE(i)                      .. SUM[j, A(i,j)*X(j)] =G= 1

MODEL SCVRPSPD /all/;
SOLVE SCVRPSPD USING LP MINIMIZING TC;
DISPLAY TC.L, X.L;

FILE problem/ setcovering.dat/;
PUT problem;
PUT TC.L /;
LOOP(j, PUT j.tl, @8, X.L(j)/);