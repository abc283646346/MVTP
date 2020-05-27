param Config{i in {1..6}};
param BP{i in {1..12}};
param Nobs == BP[12];
param Nfe == BP[11];
param R == BP[2];
param r2p == BP[3];
param f2p == BP[4];
param OBS_FULL_SCALE{i in {1..Nobs}, j in {1..Nfe}, k in {1..3}};
param wheelbase == 2.8;
param vmax == 2.5;
param amax == 0.5;
param phymax == 0.7;
param wmax == 0.5;
param tf == 40;
param hi = tf / Nfe;

var x{j in {1..Nfe}};
var y{j in {1..Nfe}};
var xf{j in {1..Nfe}};
var yf{j in {1..Nfe}};
var xr{j in {1..Nfe}};
var yr{j in {1..Nfe}};
var theta{j in {1..Nfe}};
var v{j in {1..Nfe}};
var a{j in {1..Nfe}};
var phy{j in {1..Nfe}};
var w{j in {1..Nfe}};

minimize cost_function:
hi * sum{j in {1..Nfe}}(a[j]^2 + w[j]^2);

s.t. DIFF_dxdt {i in {2..Nfe}}:
x[i] = x[i-1] + hi * v[i] * cos(theta[i]);
s.t. DIFF_dydt {i in {2..Nfe}}:
y[i] = y[i-1] + hi * v[i] * sin(theta[i]);
s.t. DIFF_dvdt {i in {2..Nfe}}:
v[i] = v[i-1] + hi * a[i];
s.t. DIFF_dthetadt {i in {2..Nfe}}:
theta[i] = theta[i-1] + hi * tan(phy[i]) * v[i] / wheelbase;
s.t. DIFF_dphydt {i in {2..Nfe}}:
phy[i] = phy[i-1] + hi * w[i];

s.t. Bonds_phy {i in {1..Nfe}}:
-phymax <= phy[i] <= phymax;
s.t. Bonds_a {i in {1..Nfe}}:
-amax <= a[i] <= amax;
s.t. Bonds_v {i in {1..Nfe}}:
-vmax <= v[i] <= vmax;
s.t. Bonds_w {i in {1..Nfe}}:
-wmax <= w[i] <= wmax;

s.t. RELATIONSHIP_XF {i in {1..Nfe}}:
xf[i] = x[i] + f2p * cos(theta[i]);
s.t. RELATIONSHIP_YF {i in {1..Nfe}}:
yf[i] = y[i] + f2p * sin(theta[i]);
s.t. RELATIONSHIP_XR {i in {1..Nfe}}:
xr[i] = x[i] + r2p * cos(theta[i]);
s.t. RELATIONSHIP_YR {i in {1..Nfe}}:
yr[i] = y[i] + r2p * sin(theta[i]);

s.t. EQ_init_x:
x[1] = Config[1];
s.t. EQ_init_y:
y[1] = Config[2];
s.t. EQ_init_theta:
theta[1] = Config[3];

s.t. EQ_init_v:
v[1] = 0;
s.t. EQ_init_a:
a[1] = 0;
s.t. EQ_init_phy:
phy[1] = 0;
s.t. EQ_init_w:
w[1] = 0;

s.t. EQ_end_x:
x[Nfe] = Config[4];
s.t. EQ_end_y:
y[Nfe] = Config[5];
s.t. EQ_end_theta:
theta[Nfe] = Config[6];

s.t. EQ_end_v:
v[Nfe] = 0;
s.t. EQ_end_a:
a[Nfe] = 0;
s.t. EQ_end_phy:
phy[Nfe] = 0;
s.t. EQ_end_w:
w[Nfe] = 0;

### Collision Avoidance Type 1 ###
s.t. ObsToFrontDics {i in {1..Nobs}, j in {1..Nfe}}:
(xf[j] - OBS_FULL_SCALE[i,j,1])^2 + (yf[j] - OBS_FULL_SCALE[i,j,2])^2 >= (R + OBS_FULL_SCALE[i,j,3])^2;

s.t. ObsToRearDics {i in {1..Nobs}, j in {1..Nfe}}:
(xr[j] - OBS_FULL_SCALE[i,j,1])^2 + (yr[j] - OBS_FULL_SCALE[i,j,2])^2 >= (R + OBS_FULL_SCALE[i,j,3])^2;

data;
param: Config := include Config;
param: OBS_FULL_SCALE := include OBS_FULL_SCALE;
param: BP := include BP;