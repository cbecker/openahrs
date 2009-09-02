load kaltest;
hold off;
for i = 1:3
	subplot(3,1,i);
	plot(in_angles(i,:));hold on;plot(angles(i,:),'r');
end

figure;
for i = 1:3
	subplot(3,1,i);
	plot(X(i+4,:));
end


