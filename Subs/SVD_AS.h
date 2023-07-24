//
// Singular Value Decomposition 
//
// Source of code : Numerical Recipies by W. Press, B.P. Flannery et al.
//                  (C) Cambridge University Press 1986 and 3rd Ed. 2007.
//
// I have made some minor changes to fit in with my codes and data types
// the changes are as indicated below
//
// A. Shepherd 20/05/2020
#include "../Subs/MY_NR3.h"
//
class SVD_AS
{
	MatDoub		a;				// the input matrix to be decomposed? actually it is u[][]
	Int			m;				// Leading dimension of A = column depth = number of equations
	Int			n;				// Row length = number of unknowns
	MatDoub		u;				// The Left Vectors after decomposition
	MatDoub		v;				// the right vectors after decomposition
	VecDoub		w;				// the singular values in 
	Doub		eps;
	Doub		tsh;
	Bool		Built;

/*	SVD_AS(MatDoub_I &a) : m(a.nrows()), n(a.ncols()), u(a), v(n,n), w(n) {
		eps = numeric_limits<Doub>::epsilon();
		decompose();
		reorder();
		tsh = 0.5*sqrt(m+n+1.)*w[0]*eps;
	}
*/
public:

	SVD_AS();

	int GetSingularValuesAndVectors(GBHMat A, FMat& Uvecs, FMat& Vvecs, FVec& Omega);

	void ReConstructAMatrix(GBHMat& U,FVec& W, GBHMat& V, GBHMat& ARet);


	void solve(VecDoub_I &b, VecDoub_O &x, Doub thresh=-1.0);
	void solve(MatDoub_I &b, MatDoub_O &x, Doub thresh=-1.0);

	Int rank(Doub thresh);
	Int nullity(Doub thresh);
	MatDoub range(Doub thresh);
	MatDoub nullspace(Doub thresh);

	Doub inv_condition() {
		return (w[0] <= 0. || w[n-1] <= 0.) ? 0. : w[n-1]/w[0];
	}

	void decompose();

	void reorder();

	Doub pythag(const Doub a, const Doub b);
};
