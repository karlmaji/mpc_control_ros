#include "main.h"


VectorXd osqp(SparseMatrix<double> hessian, VectorXd gradient, SparseMatrix<double> linearMatrix,
	VectorXd lowerBound, VectorXd upperBound)
{
	OsqpEigen::Solver solver;
	solver.settings()->setVerbosity(false);
	solver.settings()->setWarmStart(true);
	//solver.settings()->setMaxIteration(40000);
	solver.settings()->setPolish(true);
	//solver.settings()->setScaledTerimination(true);





	int numOfVar = linearMatrix.cols();
	int numOfCons = linearMatrix.rows();
	solver.data()->setNumberOfVariables(numOfVar);
	solver.data()->setNumberOfConstraints(numOfCons);

	if (!solver.data()->setHessianMatrix(hessian)) 1;
	if (!solver.data()->setGradient(gradient)) 1;
	if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) 1;
	if (!solver.data()->setLowerBound(lowerBound)) 1;
	if (!solver.data()->setUpperBound(upperBound))1;
	solver.initSolver();

	solver.solveProblem();

	VectorXd QPSolution;
	QPSolution = solver.getSolution();
	return QPSolution;
}
