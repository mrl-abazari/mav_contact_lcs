function [f_interp] = interpNaN(t,f)

    f_isNaN = isnan(f);

    if f_isNaN(1) == 1
        f(1) = 0;
        f_isNaN(1) = 0;
    end

    if f_isNaN(end) == 1
        f(end) = 0;
        f_isNaN(end) = 0;
    end

    f_interp = f;
    
    num = 1;
    
    for i = 1:length(f)-1

        d_NaN = f_isNaN(i+1) - f_isNaN(i);

        if d_NaN == 1
            n0(num) = i;
        elseif d_NaN == -1
            nf(num) = i+1;
            num = num + 1;
        end

    end

    for j = 1:num-1

        t0f = [t(n0(j)), t(nf(j))];
        f0f = [f(n0(j)), f(nf(j))];
        t0fq = [t(n0(j)):t(nf(j))];
        f_interp(n0(j):nf(j)) = interp1(t0f,f0f,t0fq,'pchip');

    end

end