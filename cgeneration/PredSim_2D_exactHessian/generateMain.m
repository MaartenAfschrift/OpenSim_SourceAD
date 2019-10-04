% This script generates c-code from the function foo.m that contains the 
% expression graph of function F. The c-code foo_jac will contain the 
% function F and its Jacobian in a format that can be exploited by CasADi.
% Authors: Joris Gillis and Antoine Falisse
import casadi.*
cg = CodeGenerator('foo_jac');
% arg should have the dimensions of the combined inputs of F, i.e. NX + NU
arg = SX.sym('arg',30); 
[y,a] = foo(arg);
F = Function('F',{arg},{y});
cg.add(F);
cg.add(F.jacobian())
% Generate also forward, reverse, and forward-over-reverse to use a exact
% Hessian
Fr = F.reverse(1);
cg.add(Fr);
for i=0:6
cg.add(F.forward(2^i));
cg.add(Fr.forward(2^i));
end
cg.generate();
