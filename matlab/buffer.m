function out = buffer(in)

persistent locVec;
if isempty(locVec)
    locVec = zeros(1,10);
end

persistent ii;
if isempty(ii)
    ii = 1;
end

locVec(ii) = in;
ii = ii + 1;

if ii > 10
    ii = 1;
end

out = locVec;
end