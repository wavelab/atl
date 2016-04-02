function pt = closest_point_on_line(a, b, p)
	v1 = p - a
	v2 = b - a
	t = dot(v1, v2) / (norm(v2))^2;

	if t < 0
		pt = a
		return;
	elseif t > 1
		pt = b
		return;
	end

	pt = a + t * v2
end

a = [1, 1];
b = [10, 10];
p = [5, 8];


pt = closest_point_on_line(a, b, p)


figure(1);
hold on;
plot([a(1), b(1)], [a(2), b(2)], 'b-');
plot(p(1), p(2), 'ro');
plot(pt(1), pt(2), 'go');
axis equal;
axis([0 12 0 12]);
drawnow;
pause;
