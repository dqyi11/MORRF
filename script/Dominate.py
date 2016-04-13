# whether s1 dominates s2
def dominate( s1, s2 ):
    for se1, se2 in s1, s2:
        if se1 > se2:
            return False
    return True

def non_dominate_rate( scores ):

    sol_num = scores.shape[0]
    obj_num = scores.shape[1]

    cnt = 0
    print "sol num " + str(sol_num)
    for i in range(sol_num):
        # assume i is non dominant
        non_dominated = True
        for j in range(sol_num):
            if i != j:
                print "compare " + str(scores[j,:]) + " " + str(scores[i,:])
                if dominate( scores[j,:] , scores[i,:]):
                     print "dominated"
                     # if j dominates i
                     non_dominated = False
                     break
        if non_dominated == True:
            cnt += 1
 
    return float(cnt)/float(sol_num)
