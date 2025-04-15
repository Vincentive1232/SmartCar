function f = OTSU_fun3(k)
if k < 180
    f = 0;
elseif k >= 180&k < 220
    f = (1/40.0)*k - 4.5;
else
    f = 1;
end
end