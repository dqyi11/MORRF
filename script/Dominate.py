
def dominate( s1, s2 ):
    for se1, se2 in s1, s2:
        if se1 > se2:
            return False
    return True

def non_dominate_rate( scores ):

    sol_num = scores.shape[0]
    obj_num = scores.shape[1]

    cnt = 0
    for i in range(sol_num):
        non_dominated = True
        for j in range(sol_num):
            if i != j:
                if dominate( scores[j,:] , scores[i,:]):
                     non_dominated = False
                     break
        if non_dominated == True:
            cnt += 1
 
    return float(cnt)/float(sol_num)
