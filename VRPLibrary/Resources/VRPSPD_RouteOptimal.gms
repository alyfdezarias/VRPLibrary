$title Modelo de programacion matematica para el TSPSPD

$include data.inc

OPTION OPTCR=0.001;

VARIABLES
 X(i,j) si el arco i j esta en la solucion
 D(i,j) mercancia por entregar en el arco i j
 P(i,j) mercancia por recoger en el arco i j
 TC costo de viaje
 BINARY VARIABLE X
 POSITIVE VARIABLE D
 POSITIVE VARIABLE P

EQUATIONS
 TRAVEL costo de viaje mas costo fijo
 INCLIENT(i) a cada cliente se llega una vez
 OUTCLIENT(j) de cada cliente se sale una vez
 DEPOTDELIVERY toda la mercancia para entregar se carga en el deposito
 DELIVERY(i) demanda de entrega
 DEPOTPICKUP toda la mercancia recogida se descarga en el deposito
 PICKUP(i) demanda de recogida
 LOAD(i,j) la carga del vehiculo no debe exceder la capacidad del vehiculo;

 TRAVEL                          .. TC =E= SUM[(i,j) $(NOT SAMEAS(i,j)), C(i,j)*X(i,j)];
 INCLIENT(j)                     .. SUM[i $(NOT SAMEAS(i,j)), X(i,j)]  =E= 1;
 OUTCLIENT(i)                    .. SUM[j $(NOT SAMEAS(i,j)), X(i,j)]  =E= 1;
 DEPOTDELIVERY                   .. SUM[(i,j) $(ORD(j) > 1 and ORD(i) = 1), D(i,j)] - SUM[i, E(i)] =E=0;
 DELIVERY(i) $(ORD(i) > 1)       .. SUM[j $(NOT SAMEAS(i,j)), D(j,i)] - SUM[j $(NOT SAMEAS(i,j)), D(i,j)] =E= E(i);
 DEPOTPICKUP                     .. SUM[(i,j) $(ORD(i) > 1 and ORD(j) = 1), P(i,j)] - SUM[i, R(i)] =E=0;
 PICKUP(i) $(ORD(i) > 1)         .. SUM[j $(NOT SAMEAS(i,j)), P(i,j)] -  SUM[j $(NOT SAMEAS (i,j)), P(j,i)] =E= R(i);
 LOAD(i,j) $(NOT SAMEAS(i,j))    .. P(i,j) + D(i,j) - X(i,j)*Q =L= 0

MODEL TSPSPD /all/;
SOLVE TSPSPD USING MIP MINIMIZING TC;
DISPLAY TC.L, X.L, D.L, P.L, INCLIENT.L, OUTCLIENT.L, DELIVERY.L, PICKUP.L, LOAD.L;

FILE problem/ tspspdsolution.dat/;
PUT problem;
PUT TC.L /;
LOOP((i,j), PUT i.tl, @8, j.tl, @16, X.L(i,j)/);