$title Modelo DUAL de Cubrimiento para el VRPSPD

OPTION OPTCR=0.001;

$include data.inc

VARIABLES
 Y(i) variable dual asociada a cada cliente
 Z objetivo del dual
 POSITIVE VARIABLE Y;

EQUATIONS
 OBJ objetivo del dual
 INCLUDE(j) restricciones duales asociadas a las rutas;

 OBJ                             .. Z =E= SUM[i, Y(i)];
 INCLUDE(j)                      .. SUM[i, A(i,j)*Y(i)] =L= R(j)

MODEL DUALSCVRPSPD /all/;
SOLVE DUALSCVRPSPD USING MIP MAXIMAZING Z;
DISPLAY Z.L, Y.L, Y.M, INCLUDE.M;

FILE problem/ dualsetcovering.dat/;
PUT problem;
PUT Z.L /;
LOOP(i, PUT i.tl, @8, Y.L(i)/);