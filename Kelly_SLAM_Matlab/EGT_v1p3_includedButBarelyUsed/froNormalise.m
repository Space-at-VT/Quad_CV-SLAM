function F = normaliseF(Fin)
% Normalises F so 3,3 = 1 but keeps its vec or matrix shape
s = sign(Fin(size(Fin,1),size(Fin,2)));
if s == 0

    