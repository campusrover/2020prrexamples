import trianglesolver as ts
from math import sin, cos, pi, sqrt, acos, asin, radians

def log(strng, vals):
    if True:
        return
    print(strng + " " + vals )

# d = previous state distance to target
# b = previous state bearing to target
# d_meas = new measurement of distance to target
# b_meas = new measurement of bearing to target
def kalman_update(K, state_dist, state_bear, meas_dist, meas_bear):
    d_prime = K * state_dist + (1-K) * meas_dist
    b_prime = K * state_bear + (1-K) * meas_bear
    return (d_prime, b_prime)

# a = previous state distance to target
# B = previous state bearing to target
# c = amount moved forward
# Returns (b = new distance to target, A=new bearing to target)
# Note that if no motion, bearing and distance dont change

def kalman_predict(a, B, c):
    if B < pi:
        log("m0: (a, B, c)", (a, B, c))
        return kalman_predict_fix_angles(a, B, c)
    else:
        log ("m1: (a, B, c)", (a, B, c))
        B = pi*2-B
        (tsb, tsA) = kalman_predict_fix_angles(a, B, c)
        tsA = pi*2-tsA
        return (tsb, tsA)

def kalman_predict_fix_angles(a, B, c):
    if (c == 0):
        log("E0: c == 0", ())
        return(a, B)
    elif (c < 0 and B != -pi):
        log("E1 input (a, c, B):", (a, c, B))
        c = -c
        B = pi - B
        log("E1 after a,c,B", (a, c, B))
        (tsa,tsb,tsc,tsA,tsB,tsC) = ts.solve(b=a, A=B, c=c)
        return(tsb, tsB)
    elif (B == 0):
        log("E2 B==0:", (a, B,c))
        return(0, pi)
    elif (B == 2*pi):
        log("E3 B==2pi:", (a,B,c))
        return(0, 1.5*pi)
    elif (B == pi):
        #print("B == pi")
        return(a, B)
    elif (B > pi):
        B_prime = (2*pi - B)
        if not all(x > 0 for x in (a,c,B_prime)):
            log("E4: a,c,B_prime: ", (a,c,B_prime))
            return(a, B)
        else:
            log ("E5 B > pi: (B, b_prime)", (b, B_prime))
            (tsa,tsb,tsc,tsA,tsB,tsC) = ts.solve(a=a, B=B_prime, c=c)
        return (tsb, 2*pi - tsA)
    elif not all(x > 0 for x in (a,c,B)):
            log("E5 (a,c,B", (a,c,B))
            return(a, B)
    else:
        (tsa,tsb,tsc,tsA,tsB,tsC) = ts.solve(a=a, B=B, c=c)
        log("OK before (a, B, c, tsb, tsA)", (a, B, c, tsb, tsA))
        tsA = pi-tsA
        log("OK before (a, B, c, tsb, tsA)", (a, B, c, tsb, tsA))
        return(tsb, tsA)

