$title Relajacion Lineal del modelo de programacion matematica para el VRPSPD con flota limitada y homogenea

$include data.inc

OPTION iterlim=10000;
OPTION OPTCR=0.001;

VARIABLES
 X(i,j) si el arco i j pertenece a la solucion
 D(i,j) mercancia por entregar en el arco i j
 P(i,j) mercancia por recoger en el arco i j
 TC costo de viaje
 BINARY VARIABLE X
 POSITIVE VARIABLE D
 POSITIVE VARIABLE P

EQUATIONS
 TRAVEL costo de viaje
 CLIENTS(j) cada cliente aparece en una ruta una unica vez
 ROUTES se emplean a lo sumo K rutas
 FLOW(j) conexidad del recorido
 EDGES(i,j) inclusion de arcos simetros no adyacente al deposito
 DEPOTDELIVERY toda la mercancia para entregar se carga en el deposito
 DELIVERY(i) satisfacer la demanda de entrega
 DELLOADLOWER(i,j) cota inferior para la carga de entrega
 DELLOADUPPER(i,j) cota superiro para la carga de entrega
 DEPOTPICKUP toda la mercancia recogida se descarga en el deposito
 PICKUP(i) satisfacer la demanda de recogida
 PCULOADLOWER(i,j) cota inferior para la carga de recogida
 PCULOADUPPER(i,j) cota superior para la carga de recogida
 LOAD(i,j) la carga no puede exceder la capacidad del vehiculo;

 TRAVEL                                                  .. TC =E= SUM[(i,j) $(NOT SAMEAS(i,j)), C(i,j)*X(i,j)];
 CLIENTS(j) $(ORD(j)>1)                                  .. SUM[i $(NOT SAMEAS(i,j)), X(i,j)]  =E= 1;
 ROUTES                                                  .. SUM[(i,j) $(ORD(j) > 1 and ORD(i) = 1), X(i,j)] =L= K;
 FLOW(j)                                                 .. SUM[i $(NOT SAMEAS(i,j)), X(i,j)] - SUM[i $(NOT SAMEAS(i,j)), X(j,i)] =E= 0;
 EDGES(i,j) $(ORD(i)>1 AND ORD(j)>1 AND NOT SAMEAS(i,j)) .. X(i,j) + X(j,i) =L= 1;
 DEPOTDELIVERY                                           .. SUM[(i,j) $(ORD(j) > 1 and ORD(i) = 1), D(i,j)] - SUM[i, E(i)] =E= 0;
 DELIVERY(i) $(ORD(i) > 1)                               .. SUM[j $(NOT SAMEAS(i,j)), D(j,i)] - SUM[j $(NOT SAMEAS(i,j)), D(i,j)] =E= E(i);
 DELLOADLOWER(i,j) $(NOT SAMEAS(i,j))                    .. X(i,j)*E(j) - D(i,j) =L= 0;
 DELLOADUPPER(i,j) $(NOT SAMEAS(i,j))                    .. X(i,j)*(Q-E(i)) - D(i,j) =G= 0;
 DEPOTPICKUP                                             .. SUM[(i,j) $(ORD(i) > 1 and ORD(j) = 1), P(i,j)] - SUM[i, R(i)] =E= 0;
 PICKUP(i) $(ORD(i) > 1)                                 .. SUM[j $(NOT SAMEAS(i,j)), P(i,j)] -  SUM[j $(NOT SAMEAS (i,j)), P(j,i)] =E= R(i);
 PCULOADLOWER(i,j) $(NOT SAMEAS(i,j))                    .. X(i,j)*R(i) - P(i,j) =L= 0;
 PCULOADUPPER(i,j) $(NOT SAMEAS(i,j))                    .. X(i,j)*(Q-R(j)) - P(i,j) =G= 0;
 LOAD(i,j) $(NOT SAMEAS(i,j))                            .. P(i,j) + D(i,j) - (Q-MAX(0, R(j)-E(j), E(i)-R(i)))*X(i,j) =L= 0

MODEL VRPSPD /all/;
SOLVE VRPSPD USING MIP MINIMIZING TC;
DISPLAY TC.L, X.L;

FILE problem/ strongVRPSPD.dat/;
PUT problem;
PUT TC.L /;
LOOP((i,j), PUT i.tl, @8, j.tl, @16, X.L(i,j)/);