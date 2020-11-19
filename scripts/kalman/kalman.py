import trianglesolver as ts
from math import sin, cos, pi, sqrt, acos, asin, radians, isclose

# d = previous state distance to target
# b = previous state bearing to target
# d_meas = new measurement of distance to target
# b_meas = new measurement of bearing to target

def kalman_update(d, b, d_meas, d_bear):
    K = 0.3
    d_prime = K * d + (1-K) * d_meas
    b_prime = K * b + (1-K) * d_bear
    return (d_prime, b_prime)

# a = previous state distance to target
# B = previous state bearing to target
# c = amount moved forward
# Returns (b = new distance to target, A=new bearing to target)
# Note that if no motion, bearing and distance dont change

def kalman_predict(a, B, c):
    #print(f"KP a={a} B={B} c={c}")
    if (c == 0):
        #print("c == 0")
        return(a, B)
    elif (c < 0):
        print("E0 a,c,B", a, c, B)
        c = -c
        B = pi - B
        (tsa,tsb,tsc,tsA,tsB,tsC) = ts.solve(b=a, A=B, c=c)
        return(tsb, tsB)
    elif (B == 0):
        print("E1 B==0:", a, B,c)
        return(0, pi)
    elif (B == 2*pi):
        print("E2 B==2pi:", a,B,c)
        return(0, 1.5*pi)
    elif (B == pi):
        #print("B == pi")
        return(a, B)
    elif (B > pi):
        B_prime = (2*pi - B)
        if not all(x > 0 for x in (a,c,B_prime)):
            print("e2 a,c,B_prime: ", a,c,B_prime)
            return(a, B)
        else:
            #print (f"B > pi: {B} {B_prime}")
            (tsa,tsb,tsc,tsA,tsB,tsC) = ts.solve(a=a, B=B_prime, c=c)
        return (tsb, 2*pi - tsA)
    elif not all(x > 0 for x in (a,c,B)):
            print("E1", a,c,B)
            return(a, B)
    else:
        (tsa,tsb,tsc,tsA,tsB,tsC) = ts.solve(a=a, B=B, c=c)
        return(tsb, tsA)

def null_kalman_predict(d, b, m):
    d_prime = d
    b_prime = b
    return d_prime, b_prime
