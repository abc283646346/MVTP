param BP{i in {1..12}};
param wheelbase == BP[1];
param FourSqR == 4 * ((BP[2])^2);
param R == BP[2];
param r2p == BP[3];
param f2p == BP[4];
param vmax == BP[5];
param amax == BP[6];
param phymax == BP[7];
param wmax == BP[8];
param tf == BP[9];
param Nv == BP[10];
param Nfe == BP[11];
param Nobs == BP[12];
param hi = tf / Nfe;

param BV{i in {1..Nv}, j in {1..6}};
param IND_V2V{i in {1..Nfe}, j in {1..(Nv-1)}, k in {1..(Nv-1)}};
param NUM_IND_V2V{i in {1..Nfe}, j in {1..(Nv-1)}};
param OBS_XY{i in {1..Nobs}, j in {1..Nfe}, k in {1..3}};
param IND_V2O{i in {1..Nfe}, j in {1..Nv}, k in {1..Nobs}};
param NUM_IND_V2O{i in {1..Nfe}, j in {1..Nv}};

var x{i in {1..Nv}, j in {1..Nfe}};
var y{i in {1..Nv}, j in {1..Nfe}};
var xf{i in {1..Nv}, j in {1..Nfe}};
var yf{i in {1..Nv}, j in {1..Nfe}};
var xr{i in {1..Nv}, j in {1..Nfe}};
var yr{i in {1..Nv}, j in {1..Nfe}};
var theta{i in {1..Nv}, j in {1..Nfe}};
var v{i in {1..Nv}, j in {1..Nfe}};
var a{i in {1..Nv}, j in {1..Nfe}};
var phy{i in {1..Nv}, j in {1..Nfe}};
var w{i in {1..Nv}, j in {1..Nfe}};

minimize cost_function:
hi * sum{i in {1..Nv}, j in {1..Nfe}}(a[i,j]^2 + w[i,j]^2); 

s.t. DIFF_dxdt {k in {1..Nv}, i in {2..Nfe}}:
x[k,i] = x[k,i-1] + hi * v[k,i] * cos(theta[k,i]);
s.t. DIFF_dydt {k in {1..Nv}, i in {2..Nfe}}:
y[k,i] = y[k,i-1] + hi * v[k,i] * sin(theta[k,i]);
s.t. DIFF_dvdt {k in {1..Nv}, i in {2..Nfe}}:
v[k,i] = v[k,i-1] + hi * a[k,i];
s.t. DIFF_dthetadt {k in {1..Nv}, i in {2..Nfe}}:
theta[k,i] = theta[k,i-1] + hi * tan(phy[k,i]) * v[k,i] / wheelbase;
s.t. DIFF_dphydt {k in {1..Nv}, i in {2..Nfe}}:
phy[k,i] = phy[k,i-1] + hi * w[k,i];

s.t. RELATIONSHIP_XF {k in {1..Nv}, i in {1..Nfe}}:
xf[k,i] = x[k,i] + f2p * cos(theta[k,i]);
s.t. RELATIONSHIP_YF {k in {1..Nv}, i in {1..Nfe}}:
yf[k,i] = y[k,i] + f2p * sin(theta[k,i]);
s.t. RELATIONSHIP_XR {k in {1..Nv}, i in {1..Nfe}}:
xr[k,i] = x[k,i] + r2p * cos(theta[k,i]);
s.t. RELATIONSHIP_YR {k in {1..Nv}, i in {1..Nfe}}:
yr[k,i] = y[k,i] + r2p * sin(theta[k,i]);

s.t. Bonds_phy {k in {1..Nv}, i in {1..Nfe}}:
-phymax <= phy[k,i] <= phymax;
s.t. Bonds_a {k in {1..Nv}, i in {1..Nfe}}:
-amax <= a[k,i] <= amax;
s.t. Bonds_v {k in {1..Nv}, i in {1..Nfe}}:
-vmax <= v[k,i] <= vmax;
s.t. Bonds_w {k in {1..Nv}, i in {1..Nfe}}:
-wmax <= w[k,i] <= wmax;
s.t. Bonds_x {k in {1..Nv}, i in {1..Nfe}}:
-20 <= x[k,i] <= 20;
s.t. Bonds_y {k in {1..Nv}, i in {1..Nfe}}:
-20 <= y[k,i] <= 20;

### Collision Avoidance Type 1 ###
s.t. ObsToFrontDics {i in {1..Nfe}, j in {1..Nv}, k in {1..NUM_IND_V2O[i,j]}}:
(xf[j,i] - OBS_XY[IND_V2O[i,j,k],i,1])^2 + (yf[j,i] - OBS_XY[IND_V2O[i,j,k],i,2])^2 >= (R + OBS_XY[IND_V2O[i,j,k],i,3])^2;

s.t. ObsToRearDics {i in {1..Nfe}, j in {1..Nv}, k in {1..NUM_IND_V2O[i,j]}}:
(xr[j,i] - OBS_XY[IND_V2O[i,j,k],i,1])^2 + (yr[j,i] - OBS_XY[IND_V2O[i,j,k],i,2])^2 >= (R + OBS_XY[IND_V2O[i,j,k],i,3])^2;

### Collision Avoidance Type 2 ###
s.t. VehicleItoJff {i in {1..Nfe}, j in {1..(Nv-1)}, k in {1..NUM_IND_V2V[i,j]}}:
(xf[j,i] - xf[IND_V2V[i,j,k],i])^2 + (yf[j,i] - yf[IND_V2V[i,j,k],i])^2 >= FourSqR;

s.t. VehicleItoJrr {i in {1..Nfe}, j in {1..(Nv-1)}, k in {1..NUM_IND_V2V[i,j]}}:
(xr[j,i] - xr[IND_V2V[i,j,k],i])^2 + (yr[j,i] - yr[IND_V2V[i,j,k],i])^2 >= FourSqR;

s.t. VehicleItoJfr {i in {1..Nfe}, j in {1..(Nv-1)}, k in {1..NUM_IND_V2V[i,j]}}:
(xf[j,i] - xr[IND_V2V[i,j,k],i])^2 + (yf[j,i] - yr[IND_V2V[i,j,k],i])^2 >= FourSqR;

s.t. VehicleItoJrf {i in {1..Nfe}, j in {1..(Nv-1)}, k in {1..NUM_IND_V2V[i,j]}}:
(xr[j,i] - xf[IND_V2V[i,j,k],i])^2 + (yr[j,i] - yf[IND_V2V[i,j,k],i])^2 >= FourSqR;


s.t. EQ_init_x {m in {1..Nv}}:
x[m,1] = BV[m,1];
s.t. EQ_init_y {m in {1..Nv}}:
y[m,1] = BV[m,2];
s.t. EQ_init_theta {m in {1..Nv}}:
theta[m,1] = BV[m,3];
s.t. EQ_init_v {m in {1..Nv}}:
v[m,1] = 0;
s.t. EQ_init_a {m in {1..Nv}}:
a[m,1] = 0;
s.t. EQ_init_phy {m in {1..Nv}}:
phy[m,1] = 0;
s.t. EQ_init_w {m in {1..Nv}}:
w[m,1] = 0;
s.t. EQ_end_x {m in {1..Nv}}:
x[m,Nfe] = BV[m,4];
s.t. EQ_end_y {m in {1..Nv}}:
y[m,Nfe] = BV[m,5];
s.t. EQ_end_theta {m in {1..Nv}}:
tan(theta[m,Nfe]) = tan(BV[m,6]);
#theta[m,Nfe] = BV[m,6];

s.t. EQ_end_v {m in {1..Nv}}:
v[m,Nfe] = 0;
s.t. EQ_end_a {m in {1..Nv}}:
a[m,Nfe] = 0;
s.t. EQ_end_phy {m in {1..Nv}}:
phy[m,Nfe] = 0;
s.t. EQ_end_w {m in {1..Nv}}:
w[m,Nfe] = 0;

data;
param: BV := include BV;
param: BP := include BP;
param: IND_V2V := include IND_V2V;
param: NUM_IND_V2V := include NUM_IND_V2V;
param: OBS_XY := include OBS_XY;
param: IND_V2O := include IND_V2O;
param: NUM_IND_V2O := include NUM_IND_V2O;