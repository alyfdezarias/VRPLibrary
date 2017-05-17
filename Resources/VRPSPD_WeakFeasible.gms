$title Modelo para factibilidad debil en el VRPSPD

OPTION OPTCR=0.001;

$include data.inc

VARIABLES
 X(i,k) si el cliente i pertenece a la ruta k
 AC clientes asignados
 BINARY VARIABLE X

EQUATIONS
 ASSIGNED total de clientes asignados
 CLIENTS(i) cada cliente pertenece a lo sumo a una ruta
 DELIVERY(k) la cantidad de mercancia a entregar no debe exceder la capacidad del vehiculo
 PICKUP(k) la cantidad de mercancia a recoger no debe exceder la capacidad del vehiculo;

 ASSIGNED               .. AC =E= SUM[(i,k), X(i,k)];
 CLIENTS(i)             .. SUM[k, X(i,k)] =L= 1;
 DELIVERY(k)            .. SUM[i, X(i,k)*E(i)] =L= Q(k);
 PICKUP(k)              .. SUM[i, X(i,k)*R(i)] =L= Q(k);

MODEL WFVRPSPD /all/;
SOLVE WFVRPSPD USING MIP MAXIMIZING AC;
DISPLAY X.L, DELIVERY.L, PICKUP.L;

FILE problem / weaksolution.dat/;
PUT problem;
PUT AC.L /;
LOOP((i,k), PUT i.tl, @8, k.tl, @16, X.L(i,k)/);