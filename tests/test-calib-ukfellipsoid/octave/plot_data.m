load calibtest;
hold off;
N = length(meas);

subplot(411);
title('Radius estimate. IT WONT BE THE SAME IF ITS NOT A PERFECT SPHERE');
plot( ones(N,1).*trueData(1), 'r' ); grid on; hold on;
plot( 1./state(1,:), 'b');

subplot(412);
title('X center. Blue: estimate / Red:true ');
plot( ones(N,1).*trueData(2), 'r' ); grid on; hold on;
plot( state(7,:), 'b');

subplot(413);
title('Y center. Blue: estimate / Red:true ');
plot( ones(N,1).*trueData(3), 'r' ); grid on; hold on;
plot( state(8,:), 'b');

subplot(414);
title('Z center. Blue: estimate / Red:true ');
plot( ones(N,1).*trueData(4), 'r' ); grid on; hold on;
plot( state(9,:), 'b');

