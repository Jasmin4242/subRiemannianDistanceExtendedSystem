clear all;
close all;

% In diesem Skript wird der Bellaiche Algorithmus fuer den differentiell angetriebenen
% Roboter mit Anhaenger durchgefuehrt.
% Dabei wird der Ursprung als Setpoint gewaehlt.
% Dafuer muessen z.B. die Lie brackets berechnet werden (liebracket.m / liebracket_ohne_expand.m)
% oder es muessen nonholonomic derivatives berechnet werden (get_Yjzk_p.m, get_YiYjyk_p.m, get_YlYiYjyk_p.m).

n=4;

%% Erstellen von X1 und X2
L1=sym('L1','real');
syms x1 x2 x3 x4;

%Erstellen von X1
X1_1= cos(x3);
X1_2= sin(x3);
X1_3= 0;
X1_4= (1/L1)*sin(x3-x4);

X1_vorher=[X1_1;X1_2;X1_3;X1_4];

%Erstellen von X2
X2_1= 0;
X2_2= 0;
X2_3= 1;
X2_4= 0;

X2_vorher=[X2_1;X2_2;X2_3;X2_4];

%% Auswerten am gewünschten Punkt
q=[0; 0; 0; 0];
X1_vorher_q=subs(X1_vorher,[x1 x2 x3 x4],[q(1) q(2) q(3) q(4)]);
X2_vorher_q=subs(X2_vorher,[x1 x2 x3 x4],[q(1) q(2) q(3) q(4)]);

%Koordinatentransformation, sodass bezüglich p=zeros(6,1)
X1=subs(X1_vorher,[x1 x2 x3 x4],[x1+q(1) x2+q(2) x3+q(3) x4+q(4)]);
X2=subs(X2_vorher,[x1 x2 x3 x4],[x1+q(1) x2+q(2) x3+q(3) x4+q(4)]);
p=zeros(n,1);
X10=subs(X1,[x1 x2 x3 x4],[p(1) p(2) p(3) p(4)]);
X20=subs(X2,[x1 x2 x3 x4],[p(1) p(2) p(3) p(4)]);
% --> X1_vorher_q=X10 und X2_vorher_q=X20

%% Berechnung der liebrackets --> Example 2.6
%X3=[X1,X2]
%X4=[X1,X3]
%X5=[X2,X3]
%X6=[X1,X4]
%die Vektorfelder X3 bis X6 müssten dann schon in geswitchten Koordinaten
%sein, weil ja mit bereits verschobenen X1 und X2 berechnet worden sind

ad_fng = sym('ad_fng','real');
x = [x1,x2,x3,x4];

ad_fng=liebracket(X1,X2,x,1);
X3=ad_fng(:,2);

ad_fng=liebracket(X1,X3,x,1);
X4=ad_fng(:,2);

ad_fng=liebracket(X2,X3,x,1);
X5=ad_fng(:,2);

%% liebrackets ebenso an p=zeros(6,1) auswerten
X30=subs(X3,[x1 x2 x3 x4],[p(1) p(2) p(3) p(4)]);
X40=subs(X4,[x1 x2 x3 x4],[p(1) p(2) p(3) p(4)]);

%% Bellaiche Algorithm Step 1 und 2, Berechnung der Koordinaten y
%Matrix A
A=[X10';X20';X30';X40'];
A_inv_transp=transpose(inv(A));
x = [x1;x2;x3;x4];
y_von_x=A_inv_transp*x; %WICHTIGES Ergebnis: y1=.... y2=... in Abhängigkeit von x1,x2,...
y_von_x_0=subs(y_von_x,[x1 x2 x3 x4],[p(1) p(2) p(3) p(4)]);

%umgekehrte Transformation rausfinden, x1(y1,y2,y3,y4,y5,y6)
syms y1 y2 y3 y4
eq1=y1==y_von_x(1);
eq2=y2==y_von_x(2);
eq3=y3==y_von_x(3);
eq4=y4==y_von_x(4);
x_von_y=solve([eq1,eq2,eq3,eq4],[x1 x2 x3 x4]); %WICHTIGES Ergebnis: x1=... x2=... in Abhängigkeit von y1,y2,y3,...

%damit die Vektorfelder X1 bis X6, geschrieben in den Koordinaten y, bestimmen
X1_y=subs(X1,[x1 x2 x3 x4],[x_von_y.x1 x_von_y.x2 x_von_y.x3 x_von_y.x4]);
X2_y=subs(X2,[x1 x2 x3 x4],[x_von_y.x1 x_von_y.x2 x_von_y.x3 x_von_y.x4]);
X3_y=subs(X3,[x1 x2 x3 x4],[x_von_y.x1 x_von_y.x2 x_von_y.x3 x_von_y.x4]);
X4_y=subs(X4,[x1 x2 x3 x4],[x_von_y.x1 x_von_y.x2 x_von_y.x3 x_von_y.x4]);

%Bestimmung von Y1 und Y2
%Vorgehen: nacheinander y1, y2,...y6 in den Koordinaten von x anschauen und
%dann ableiten.
%für x1_dot dann die erste Zeile von X1 bzw. X2 nehmen, aber in den
%Koordinaten y
%für x2_dot die zweite Zeile von X1_y bzw. X2_y nehmen

syms Y1 Y2
Y1=sym(zeros(n,1));
Y2=sym(zeros(n,1));
for ii=1:n
    y_zeile=y_von_x(ii);
    %jetzt die Abletung von y_zeile berechnen und damit die Einträge von Y1 und Y2 erhalten:
    Y1_zeile=subs(y_zeile,[x1 x2 x3 x4],[X1_y(1) X1_y(2) X1_y(3) X1_y(4)]); 
    Y2_zeile=subs(y_zeile,[x1 x2 x3 x4],[X2_y(1) X2_y(2) X2_y(3) X2_y(4)]);
    Y1(ii,1)=Y1_zeile;
    Y2(ii,1)=Y2_zeile;
end

%Bestimmung von Y3 Y4 Y5 Y6
%X3=[X1,X2]
%X4=[X1,X3]
%X5=[X2,X3]
%X6=[X1,X4]
syms y1 y2 y3 y4;
y=[y1 y2 y3 y4];

ad_fng = sym('ad_fng','real');

ad_fng=liebracket_ohne_expand(Y1,Y2,y,1);
Y3=ad_fng(:,2);

ad_fng=liebracket_ohne_expand(Y1,Y3,y,1);
Y4=ad_fng(:,2);

%% Bellaiche Algorithm Step3
syms x1 x2 x3 x4 
x=[x1 x2 x3 x4];

syms z1 z2 z3 z4;

%Bestimmung von z1,z2,z3
z1_von_y=y1;
z2_von_y=y2;
z3_von_y=y3;

%Bestimmung von z4
% Y1Y2y4(x)(p)
vektorfeld_rechts=Y2;
vektorfeld_links=Y1;
koordinate_nummer=4; %y4
Y1Y2y4_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,y,p,n); %=0
% Y1Y1y4(x)(p)
vektorfeld_rechts=Y1;
vektorfeld_links=Y1;
koordinate_nummer=4; %y4
Y1Y1y4_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,y,p,n);
% Y2Y2y4(x)(p)
vektorfeld_rechts=Y2;
vektorfeld_links=Y2;
koordinate_nummer=4; %y4
Y2Y2y4_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,y,p,n);
%z4
z4_von_y=y4-Y1Y2y4_p*y1*y2-Y1Y1y4_p*0.5*y1*y1-Y2Y2y4_p*0.5*y2*y2;

%z1,...,z4 in Abhängigkeit von x WICHTIG
z1_von_x=subs(z1_von_y,[y1 y2 y3 y4],[y_von_x(1) y_von_x(2) y_von_x(3) y_von_x(4)]);
z2_von_x=subs(z2_von_y,[y1 y2 y3 y4],[y_von_x(1) y_von_x(2) y_von_x(3) y_von_x(4)]);
z3_von_x=subs(z3_von_y,[y1 y2 y3 y4],[y_von_x(1) y_von_x(2) y_von_x(3) y_von_x(4)]);
z4_von_x=subs(z4_von_y,[y1 y2 y3 y4],[y_von_x(1) y_von_x(2) y_von_x(3) y_von_x(4)]);

z_von_x=[z1_von_x;z2_von_x;z3_von_x;z4_von_x];

%umgekehrte Transformation rausfinden, x1(z1,z2,z3,z4)
syms z1 z2 z3 z4 
eq1=z1==z1_von_x;
eq2=z2==z2_von_x;
eq3=z3==z3_von_x;
eq4=z4==z4_von_x;
x_von_z=solve([eq1,eq2,eq3,eq4],[x1 x2 x3 x4]); %WICHTIGES Ergebnis: x1=... x2=... in Abhängigkeit von z1,z2,z3,...

%damit die Vektorfelder X1 bis X6, geschrieben in den Koordinaten z, bestimmen
X1_z=subs(X1,[x1 x2 x3 x4],[x_von_z.x1 x_von_z.x2 x_von_z.x3 x_von_z.x4]);
X2_z=subs(X2,[x1 x2 x3 x4],[x_von_z.x1 x_von_z.x2 x_von_z.x3 x_von_z.x4]);
X3_z=subs(X3,[x1 x2 x3 x4],[x_von_z.x1 x_von_z.x2 x_von_z.x3 x_von_z.x4]);
X4_z=subs(X4,[x1 x2 x3 x4],[x_von_z.x1 x_von_z.x2 x_von_z.x3 x_von_z.x4]);

%Bestimmung von Z1 und Z2
%Vorgehen: nacheinander z1, z2,...z6 in den Koordinaten von x anschauen und
%dann ableiten.
%für x1_dot dann die erste Zeile von X1 bzw. X2 nehmen, aber in den
%Koordinaten z
%für x2_dot die zweite Zeile von X1_z bzw. X2_z nehmen

syms Z1 Z2
Z1=sym(zeros(n,1));
Z2=sym(zeros(n,1));
for ii=1:n
    z_zeile=z_von_x(ii); %Produktregel?! Z1 und Z2 stimmen nicht!
    %jetzt die Abletung von z_zeile berechnen und damit die Einträge von Y1 und Y2 erhalten:
    Z1_zeile=subs(z_zeile,[x1 x2 x3 x4],[X1_z(1) X1_z(2) X1_z(3) X1_z(4)]); 
    Z2_zeile=subs(z_zeile,[x1 x2 x3 x4],[X2_z(1) X2_z(2) X2_z(3) X2_z(4)]);
    Z1(ii,1)=Z1_zeile;
    Z2(ii,1)=Z2_zeile;
end

%Bestimmung von Z3 Z4 Z5 Z6
%X3=[X1,X2]
%X4=[X1,X3]
%X5=[X2,X3]
%X6=[X1,X4]
syms z1 z2 z3 z4;
z=[z1 z2 z3 z4];

ad_fng = sym('ad_fng','real');

ad_fng=liebracket_ohne_expand(Z1,Z2,z,1);
Z3=ad_fng(:,2);

ad_fng=liebracket_ohne_expand(Z1,Z3,z,1);
Z4=ad_fng(:,2);

%% Ordnung von z1, z2,...,z6
%Ordnung von z1
koordinate_nummer=1; %z1
%z1(p)=0 --> z1>=1
%Z1z1(p),Z2z1(p)
vektorfeld_rechts=Z1;
Z1z1_p =  get_Yjyk_p(vektorfeld_rechts,koordinate_nummer,z,p); %=1 daher Ordnung 1

%Ordnung von z2
koordinate_nummer=2; %z2
%z2(p)=0 --> Ordnung>=1
%Z1z2(p),Z2z2(p)
vektorfeld_rechts=Z1;
Z1z2_p =  get_Yjyk_p(vektorfeld_rechts,koordinate_nummer,z,p); %=0 
vektorfeld_rechts=Z2;
Z2z2_p =  get_Yjyk_p(vektorfeld_rechts,koordinate_nummer,z,p); %=1 daher Ordnung 1

%Ordnung von z3
koordinate_nummer=3; %z3
%z3(p)=0 --> Ordnung>=1
%Z1z3(p),Z2z3(p)
vektorfeld_rechts=Z1;
Z1z3_p =  get_Yjyk_p(vektorfeld_rechts,koordinate_nummer,z,p); %=0 
vektorfeld_rechts=Z2;
Z2z3_p =  get_Yjyk_p(vektorfeld_rechts,koordinate_nummer,z,p); %=0 daher Ordnung >=2
%Z1Z1z3(p),Z2Z1z3(p),Z1Z2z3(p),Z2Z2z3(p)
% Z1Z1z3(p)
vektorfeld_rechts=Z1;
vektorfeld_links=Z1;
Z1Z1z3_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,z,p,n); %=0
%Z2Z1z3(p)
vektorfeld_rechts=Z2;
vektorfeld_links=Z1;
Z1Z2z3_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,z,p,n); %=0
%Z1Z2z3(p)
vektorfeld_rechts=Z1;
vektorfeld_links=Z2;
Z2Z1z3_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,z,p,n); %=-1
%Z2Z2z3(p)
vektorfeld_rechts=Z2;
vektorfeld_links=Z2;
Z2Z2z3_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,z,p,n); %


%Ordnung von z4
koordinate_nummer=4; %z4
%z4(p)=0 --> Ordnung>=1
%Z1z4(p),Z2z4(p)
vektorfeld_rechts=Z1;
Z1z4_p =  get_Yjyk_p(vektorfeld_rechts,koordinate_nummer,z,p); %=0 
vektorfeld_rechts=Z2;
Z2z4_p =  get_Yjyk_p(vektorfeld_rechts,koordinate_nummer,z,p); %=0 daher Ordnung >=2
%Z1Z1z4(p),Z2Z1z4(p),Z1Z2z4(p),Z2Z2z4(p)
% Z1Z1z4(p)
vektorfeld_rechts=Z1;
vektorfeld_links=Z1;
Z1Z1z4_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,z,p,n); %=0
%Z2Z1z4(p)
vektorfeld_rechts=Z2;
vektorfeld_links=Z1;
Z1Z2z4_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,z,p,n); %=0
%Z1Z2z3(p)
vektorfeld_rechts=Z1;
vektorfeld_links=Z2;
Z2Z1z4_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,z,p,n); %=-1
%Z2Z2z3(p)
vektorfeld_rechts=Z2;
vektorfeld_links=Z2;
Z2Z2z4_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,z,p,n); %
%Z1Z1Z1z4_p, Z2Z1Z1z4_p, Z1Z2Z1z4_p,Z2Z2Z1z4_p
funktion=z4;
%Z1Z1Z1z4_p
vektorfeld_rechts=Z1;
vektorfeld_links=Z1;
vektorfeld_ganz_links=Z1;
Z1Z1Z1z4_p=  get_YlYiYjyk_p(vektorfeld_rechts,vektorfeld_links,vektorfeld_ganz_links,funktion,z,p,n); %0
%Z2Z1Z1z4_p
vektorfeld_rechts=Z1;
vektorfeld_links=Z1;
vektorfeld_ganz_links=Z2;
Z2Z1Z1z4_p=  get_YlYiYjyk_p(vektorfeld_rechts,vektorfeld_links,vektorfeld_ganz_links,funktion,z,p,n); %1
%Z1Z2Z1z4_p
vektorfeld_rechts=Z1;
vektorfeld_links=Z2;
vektorfeld_ganz_links=Z1;
Z1Z2Z1z4_p=  get_YlYiYjyk_p(vektorfeld_rechts,vektorfeld_links,vektorfeld_ganz_links,funktion,z,p,n); %#
%Z2Z2Z1z4_p
vektorfeld_rechts=Z1;
vektorfeld_links=Z2;
vektorfeld_ganz_links=Z2;
Z2Z2Z1z4_p=  get_YlYiYjyk_p(vektorfeld_rechts,vektorfeld_links,vektorfeld_ganz_links,funktion,z,p,n); %

%% Taylor Expansion Z1 und Z2
Z1_taylor_1=taylor(Z1(1),[z1 z2 z3 z4],[0 0 0 0],'Order', 4) %dann bis Terme hoch 3
Z1_taylor_2=taylor(Z1(2),[z1 z2 z3 z4],[0 0 0 0],'Order', 4)
Z1_taylor_3=taylor(Z1(3),[z1 z2 z3 z4],[0 0 0 0],'Order', 4)
Z1_taylor_4=taylor(Z1(4),[z1 z2 z3 z4],[0 0 0 0],'Order', 4)

%% verify that it is truly nilpotent
Z1_n=[1;0;-z2;-z3];
Z2_n=[0;1;0;0];

Z3_n=liebracket_ohne_expand(Z1_n,Z2_n,z,1);
Z3_n=Z3_n(:,2);
Z4_n=liebracket_ohne_expand(Z1_n,Z3_n,z,1);
Z4_n=Z4_n(:,2);

Z5_n=liebracket_ohne_expand(Z2_n,Z3_n,z,1);
Z5_n=Z5_n(:,2); %Nullvektor

Z6_n=liebracket_ohne_expand(Z1_n,Z4_n,z,1);
Z6_n=Z6_n(:,2); %Nullvektor

%% Beweis Residuum
alpha=sym('alpha','real');
term_in_R4=sin(alpha*z2)-sin(alpha*z2+alpha^2*z3+alpha^3*z4)+alpha^2*z3;
Zterm_in_R4_taylor=taylor(term_in_R4,[z1 z2 z3 z4],[0 0 0 0],'Order', 4);