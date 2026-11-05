function DH_matrix=DenavitH(th, d_val, a_val, alp)
DH_matrix=[
    cos(th), -sin(th)*cos(alp),  sin(th)*sin(alp), a_val*cos(th);
    sin(th),  cos(th)*cos(alp), -cos(th)*sin(alp), a_val*sin(th);
    0,                sin(alp),          cos(alp),         d_val;
    0,                       0,                 0,             1];
end