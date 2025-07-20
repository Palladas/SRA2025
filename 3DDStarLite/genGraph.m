load("nodesVisData.mat","times");
x = (0:50).';
f = fit(x,times,"poly2");
y = f(x);
plot(x, times, 'o', x, y, '-');
xlabel('X values');
ylabel('Times');
title('Fitted Polynomial Curve');