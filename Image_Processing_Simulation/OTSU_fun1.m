function f = OTSU_fun1(k)
if k < 70
    f = 1;
elseif k >= 70&k<110
    f = -(1/40.0)*k + 2.75;
else
    f = 0;
end
end
       