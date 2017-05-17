$title Modelo de programacion matematica para el VRPSPD con flota limitada y heterogenea considerando factibilidad debil

$include data.inc

OPTION OPTCR=0.01;

VARIABLES
 X(i,j,k) si el arco i j es recorrido por el vehiculo k
 D(i,j) mercancia por entregar en el arco i j
 P(i,j) mercancia por recoger en el arco i j
 TC costo de viaje
 BINARY VARIABLE X
 POSITIVE VARIABLE D
 POSITIVE VARIABLE P

EQUATIONS
 TRAVEL costo de viaje
 CLIENTS(j) cada cliente aparece en una ruta una unica vez
 ROUTES(k) cada vehiculo se emplea a lo sumo 1 vez
 FLOW(j,k) conservacion de flujo si un cliente es atendido por el vehiculo k este tiene q continuar el recorrido a partir de dicho clietne
 DEPOTDELIVERY toda la mercancia para entregar se carga en el deposito
 DELIVERY(i) satisfacer la demanda de entrega
 DEPOTPICKUP toda la mercancia recogida se descarga en el deposito
 PICKUP(i) satisfacer la demanda de recogida
 DELIVERYLOAD(i,j) la carga por entregar no puede exceder la capacidad del vehiculo
 PICKUPLOAD(i,j) la carga recogida no puede exceder la capacidad del vehiculo;

 TRAVEL                                  .. TC =E= SUM[(i,j,k) $(NOT SAMEAS(i,j)), C(i,j)*X(i,j,k)];
 CLIENTS(j) $(ORD(j) > 1)                .. SUM[(i,k) $(NOT SAMEAS(i,j)), X(i,j,k)]  =E= 1;
 ROUTES(k)                               .. SUM[(i,j) $(ORD(j) > 1 and ORD(i) = 1), X(i,j,k)] =L= 1;
 FLOW(j,k)                               .. SUM[i $(NOT SAMEAS(i,j)), X(i,j,k)] - SUM[i $(NOT SAMEAS(i,j)), X(j,i,k)] =E= 0;
 DEPOTDELIVERY                           .. SUM[(i,j) $(ORD(j) > 1 and ORD(i) = 1), D(i,j)] - SUM[i, E(i)] =E= 0;
 DELIVERY(i) $(ORD(i) > 1)               .. SUM[j $(NOT SAMEAS(i,j)), D(j,i)] - SUM[j $(NOT SAMEAS(i,j)), D(i,j)] =E= E(i);
 DEPOTPICKUP                             .. SUM[(i,j) $(ORD(i) > 1 and ORD(j) = 1), P(i,j)] - SUM[i, R(i)] =E= 0;
 PICKUP(i) $(ORD(i) > 1)                 .. SUM[j $(NOT SAMEAS(i,j)), P(i,j)] -  SUM[j $(NOT SAMEAS (i,j)), P(j,i)] =E= R(i);
 DELIVERYLOAD(i,j) $(NOT SAMEAS(i,j))    .. D(i,j) - SUM[k, X(i,j,k)*Q(k)] =L= 0;
 PICKUPLOAD(i,j) $(NOT SAMEAS(i,j))      .. P(i,j) - SUM[k, X(i,j,k)*Q(k)] =L= 0

MODEL VRPSPD /all/;
SOLVE VRPSPD USING MIP MINIMIZING TC;
DISPLAY TC.L, X.L, D.L, P.L, ROUTES.L, CLIENTS.L, FLOW.L, DELIVERY.L, PICKUP.L, DELIVERYLOAD.L, PICKUPLOAD.L;

FILE problem/ spdweaksolution.dat/;
PUT problem;
PUT TC.L /;
LOOP((i,j,k), PUT i.tl, @8, j.tl, @16, k.tl, @20, X.L(i,j,k)/);