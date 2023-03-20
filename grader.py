import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV):
    # problems = [["./map1.txt", "pi/2,pi/4,pi/2,0.785398,1.570796",
    #                             "0.392699,2.356194,pi,2.8274328,pi*3/2"],
    #         ["./map2.txt", "0.392699,2.356194,3.141592",
    #                             "1.570796,0.785398,1.570796"]]
    


    problems = [["./map2.txt", "0.525268,2.03899,1.71478,3.09235,1.01923", "0.808671,2.64286,1.41252,0.286443,6.02491" ],
                ["./map2.txt", "0.530504,2.20682,1.61026,3.91841,3.77793", "0.379104,1.96982,5.72596,6.08189,2.38171" ],
                ["./map2.txt", "0.837881,2.00828,4.48953,1.87232,2.04508", "0.279566,2.63827,0.854639,0.132182,5.56061" ],
                ["./map2.txt", "1.07808,2.64283,0.745988", "0.581257,1.73064,5.06792"  ],
                ["./map2.txt", "0.986684,1.04509,5.93496", "0.664134,2.38453,2.54823"]]
                

    scores = []
    for aPlanner in [0, 1, 2, 3]:
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "tmp.txt"
            startPosString = ",".join(convertPIs(startPos))
            goalPosString = ",".join(convertPIs(goalPos))
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPosString, goalPosString,
                aPlanner, outputSolutionFile)
            commandVerify = "./verifier.out {} {} {} {} {}".format(
                inputMap, numDOFs, startPosString, goalPosString,
                outputSolutionFile)
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                ### Visualize their results
                commandViz = "python visualizer.py tmp.txt --gifFilepath=grades_{}.gif".format(i)
                commandViz += " --incPrev=1"
                subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./planner.out", "test.csv")