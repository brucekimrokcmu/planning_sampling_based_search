/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <ostream>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

/* Include custom classes */
#include "PRMSolver.hpp"
#include "RRTSolver.hpp"
#include "Utils.hpp"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
// // PRM
// {
// 	//no plan by default
// 	*plan = NULL;
// 	*planlength = 0;
// 	std::vector<std::vector<double>> path;
// 	int numOfSamples = 5000;
// 	const double MAXDIST_THRESHOLD = PI;
// 	// std::cout<<"Instantiating PRMSolver class"<<std::endl;
// 	PRMSolver prm(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, numOfSamples, MAXDIST_THRESHOLD);
// 	std::cout<<"Calling BUildRoadMap function"<<std::endl;
// 	std::unique_ptr<Graph> graphSmartPtr = prm.InitializeGraph();
// 	prm.BuildRoadMap(graphSmartPtr);
	
// 	std::cout<<"Query Starts"<<std::endl;
// 	path = prm.QueryRoadMap(graphSmartPtr);

// 	*plan = convert2DVectorTo2DArray(path);
// 	*planlength = path.size();


//     return;
// }

// //// RRT
// {
// 	//no plan by default
// 	*plan = NULL;
// 	*planlength = 0;
// 	std::vector<std::vector<double>> path;
// 	const double eps = 0.2*PI;
// 	const double stepIters = 50;
// 	const double goalTol = 1.0;
// 	const int maxIters = 10000;

// 	// RRTSolver::RRTSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, double eps, double goalTol, int maxIters)
// 	RRTSolver rrt(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, eps, stepIters, goalTol, maxIters);
// 	Tree mmyTree = rrt.BuildRRT();
// 	std::cout<<"Tree size: " << mmyTree.GetTree().size() << std::endl; //TOO SMALL!
	
// 	path = mmyTree.GetPath(mmyTree.GetTree().back());

// 	*plan = convert2DVectorTo2DArray(path);
// 	*planlength = path.size();

//     return;
// }

// /// RRTCONNECT
// {
// 	//no plan by default
// 	*plan = NULL;
// 	*planlength = 0;
// 	std::vector<std::vector<double>> path;
// 	const double eps = 0.2;
// 	const double stepIters = 20000;
// 	const double goalTol = 0.001;
// 	const int maxIters = 40000;

// 	// RRTSolver::RRTSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, double eps, double goalTol, int maxIters)
// 	RRTSolver rrt(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, eps, stepIters, goalTol, maxIters);
// 	path = rrt.BuildRRTConnect();	

// 	for (auto& p : path){
// 		for (int i=0; i<p.size(); i++){
// 			std::cout<< p[i]<<" ";
// 		}
// 		printf("\n");
// 	}

// 	*plan = convert2DVectorTo2DArray(path);
// 	*planlength = path.size();

//     return;

// }

/// RRT*
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
	std::vector<std::vector<double>> path;
	const double eps = 0.2;
	const double stepIters = 50;
	const double goalTol = 1.0;
	const int maxIters = 1000;


	// RRTSolver::RRTSolver(double* map, int maxX, int maxY, double* startPos, double* goalPos, const int numOfDOFs, double eps, double goalTol, int maxIters)
	RRTSolver rrt(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, eps, stepIters, goalTol, maxIters);
	Tree myRRTStarTree = rrt.BuildRRTStar();	


	std::cout<<"Tree size: " << myRRTStarTree.GetTree().size() << std::endl; //TOO SMALL!
	
	path = myRRTStarTree.GetPath(myRRTStarTree.GetTree().back());

	for (auto& p : path){
		for (int i=0; i<p.size(); i++){
			std::cout<< p[i]<<" ";
		}
		std::cout<< "path size: " << path.size() << std::endl;
	}

	path.push_back(convertToVector(armgoal_anglesV_rad, numofDOFs));	

	*plan = convert2DVectorTo2DArray(path);
	*planlength = path.size();

    return;

}

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */

int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL; 	
	int planlength = 0;
	planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);	
	

	// std::vector<std::vector<double>> plan;
	// int planlength = 0;
	// planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, plan, planlength);

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
