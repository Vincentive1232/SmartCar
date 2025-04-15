function f = OTSU_fun2(k)
if k < 70
    f = 0;
elseif k >= 70&k < 110
    f = (1/40.0)*k - 1.75;
elseif k >= 110&k < 180
    f = 1;
elseif k >= 180&k < 220
    f = -(1/40.0)*k + 5.5;
else
    f = 0;
end
end