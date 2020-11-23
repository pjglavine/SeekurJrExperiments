function Q=skewql(quat)

a=quat(1);
q=[quat(2) quat(3) quat(4)]';

Q=a*eye(4)+[0 -q';q skew(q)];

end