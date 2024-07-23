distance = [160 200 250 275 300 350 400 450];
boxX = [305 215 157 140 125 105 90 85];
plot(boxX, distance)
hold on;
plot(boxX, distance, 'rx')
hold on;
# coeff = polyfit(boxX,distance,2);
# x = 80:350;
# y = coeff(1)*(x.^2) + coeff(2)*x + coeff(3);
# coeff = polyfit(boxX,distance,3);
# x = 80:350;
# y = coeff(1)*(x.^3) + coeff(2)*(x.^2) + coeff(3)*x + coeff(4);
# coeff = polyfit(boxX,distance,4)
# x = 80:350;
# y = coeff(1)*(x.^4) + coeff(2)*(x.^3) + coeff(3)*(x.^2) + coeff(4)*x + coeff(5)
# coeff = polyfit(boxX,distance,7)
# x = 80:350;
# y = coeff(1)*(x.^7) + coeff(2)*(x.^6) + coeff(3)*(x.^5) + coeff(4)*(x.^4) + coeff(5)*(x.^3) + coeff(6)*(x.^2) + coeff(7)*x + coeff(8)
coeff1 = polyfit(boxX(1:3),distance(1:3),3);
x1 = 157:350;
y1 = coeff1(1)*(x1.^3) + coeff1(2)*(x1.^2) + coeff1(3)*x1 + coeff1(4);
# y1 = coeff1(1)*(x1.^4) + coeff1(2)*(x1.^3) + coeff1(3)*(x1.^2) + coeff1(4)*x1 + coeff1(5);
coeff2 = polyfit(boxX(3:8),distance(3:8),3);
x2 = 80:157;
y2 = coeff2(1)*(x2.^3) + coeff2(2)*(x2.^2) + coeff2(3)*x2 + coeff2(4);
# y2 = coeff2(1)*(x2.^4) + coeff2(2)*(x2.^3) + coeff2(3)*(x2.^2) + coeff2(4)*x2 + coeff2(5);
plot(x1,y1)
hold on
plot(x2,y2)
