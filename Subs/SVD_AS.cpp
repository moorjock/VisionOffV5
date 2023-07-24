//
// Source of code : Numerical Recipies by W. Press, B.P. Flannery et al.
//                  (C) Cambridge University Press 1986 and 3rd Ed. 2007.
//
// I have made some minor changes to fit in with my codes and data types
// the changes are as indicated below
//
// A. Shepherd 20/05/2020
//
using namespace std;

#include <cstdio>		// fopen and file read/write operations
#include <cmath>		// standard maths functions sin/cos/log etc.
#include <climits>		// definitions like DBL_EPSILON and not much else
#include <cstdlib>		// other standard functions like atoi max,min etc

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

//
#include "MY_NR3.h"

#include "device_funcs.cuh"

#include "SVD_AS.h"



SVD_AS::SVD_AS()	
{
	Built=false;
	u.resize(3,3);
}
/*
void SVD_AS::Construct(GBHMat CopA, int ReBuild)
{
// This relocates code from the original constructor
	int i,j;
	
	u = aa;				// u is the matrix actually worked on in decompose()
	eps = __DBL_EPSILON__;
	m = u.nrows();
	n = u.ncols();
	v.resize(n,n);
	w.resize(n);
}
*/

int SVD_AS::GetSingularValuesAndVectors(GBHMat A, FMat& Uvecs, FMat& Vvecs, FVec& Omega)
{
//
// This is my interface routine to the SVD routines from the original code
// basically to keep my codes as clean and linkage-free as possible
// 
	int i, j, M, N;
	M = 3;
	N = 3;

	for (i = 0; i < M; i++) {
		for (j = 0; j < N; j++) {
			u[i][j] = A(i,j);
		}
	}
	eps = __DBL_EPSILON__;
	m = u.nrows();	// M
	n = u.ncols();
	v.resize(n,n);
	w.resize(n);
//
// This does the SVD decomposition
// and reorder of vectors
//
	decompose();
	reorder();
	tsh = 0.5*sqrt(m+n+1.)*w[0]*eps;
//
// port the results into my matrix and vector classes
// respecting the original data, but trimming residuals
//
	for (i = 0; i < n; i++) {
		Omega(i) = w[i];
		if (Omega(i) < tsh) {
			Omega(i)=0.0;
		}

		for (j = 0; j < m; j++) {
			Uvecs(j,i) = float(u[j][i]);
		}

		for (j = 0; j < n; j++) {
			Vvecs(j,i) = float(v[j][i]);
		}
	}

	return 1;
}
//
//-------------------------------------------------------------------------------------
//
void  SVD_AS::ReConstructAMatrix(GBHMat& U, FVec& W, GBHMat& VT, GBHMat & ARet)
{
//
// Takes the Input decomposition Matrices U,W,VT and reconstructs the 
// linear projection ARet. This can be smaller than the original matrix 
// dimensions depending on the number of non-zero singular values in W
// W is assumed to be in descending order with zeroed values for
// singular values less than threshold, thus the reconstruction
// can be a lower dimension than the input matrices
// 
//
	GBHMat T1;
	int i,j,k, M, N, N2;

	M = 4;	// U.rows();
	N = 4;	// U.cols();
	N2=N;
	for (i = 0; i < N; i++) {
		if (fabs(W(i)) <= 0.0) {
			N2=i;	// first zero sets N2
			break;
		}
	}
//
//
// The sub-set N2 is now the principle dimension
// compute [W][VT]
	T1.InitMatrix();

	for (i = 0; i < N2; i++) {
		for (j = 0; j < N; j++) {
			T1(i,j)=W(i)*VT(i,j);
		}
	}
//
// Note also that M can be far greater than N or N2
//
	ARet.InitMatrix();

	for(i=0; i<M; i++) {
		for(j=0; j<N; j++) {
			ARet(i,j) = 0.0;
			for (k=0; k<N2; k++) {
				ARet(i,j) += U(i,k) * T1(k,j);
			}
		}
	}
}
//
//-------------------------------------------------------------------------------------
//
void SVD_AS::decompose() 
{
/*
C Singular Value Decomposition
C
C Note that the CASE of variable names has changed since 1986 ed.
C the code has also changed a bit to reflect C/C++ conventions but
C is still very similar to the original FORTRAN of 1986 edition.
C Note though the substitution of A with U in this C/C++ code.
C I have augmented the code a bit by indenting loops that were
C previously 1-line statements, this makes the code clearer
C to read and maps more closely to the FORTRAN. 
C
C  Arguments are generated via the Construct() call above but :
C
C       A (now U) R(M,N)          General input matrix to be inverted
C       M         I               Actual row dimension this call
C       N         I               Actual column dimension this call
C       W         R(N)            Diag vector of singular values of A
C       V         R(N,N)          decomposition matrix V right vectors
C       U                         decomposition matrix U left vectors
C
C Source of code : Numerical Recipies by W. Press, B.P. Flannery et al.
C                  (C) Cambridge University Press 1986 and 3rd Ed. 2007.
C User Notes:
C
C  Given matrix A(U) with logical dimensions M,N
C  this function computes its singular value decomposition A = U*W*V^t
C  The diagonal matrix of singular values W is output as the vector W. 
C  The matrix V (not the transpose V^t) is output as V. 
C
C  M must be greater than or equal to N 
C  if it is smaller then A should be filled up to N (ie square) by adding 
C  additional zero rows. These additional rows will produce additional 
C  low (zero) condition numbers but are nessasary for correct operation of the code.
C
*/
	bool flag;
	Int i,its,j,jj,k,l,nm;
	Doub anorm,c,f,g,h,s,scale,x,y,z;
	VecDoub rv1(n);
//
// Householder reduction to bi-diagonal form
//
	g = scale = anorm = 0.0;
	for (i=0;i<n;i++) {
		l=i+2;
		rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i < m) {
			for (k=i;k<m;k++){
				scale += abs(u[k][i]);
			}
			if (scale != 0.0) {
				for (k=i;k<m;k++) {
					u[k][i] /= scale;
					s += u[k][i]*u[k][i];
				}
				f=u[i][i];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				u[i][i]=f-g;
				for (j=l-1;j<n;j++) {
					for (s=0.0,k=i;k<m;k++){
						s += u[k][i]*u[k][j];
					}
					f=s/h;
					for (k=i;k<m;k++){
						u[k][j] += f*u[k][i];
					}
				}
				for (k=i;k<m;k++){
					u[k][i] *= scale;
				}
			}
		}
		w[i]=scale *g;
		g=s=scale=0.0;
		if ( (i+1 <= m) && (i+1 != n) ) {
			for (k=l-1;k<n;k++) {
				scale += abs(u[i][k]);
			}
			if (scale != 0.0) {
				for (k=l-1;k<n;k++) {
					u[i][k] /= scale;
					s += u[i][k]*u[i][k];
				}
				f=u[i][l-1];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				u[i][l-1]=f-g;
				for (k=l-1;k<n;k++){
					rv1[k]=u[i][k]/h;
				}
				for (j=l-1;j<m;j++) {
					for (s=0.0,k=l-1;k<n;k++){
						s += u[j][k]*u[i][k];
					}
					for (k=l-1;k<n;k++){
						u[j][k] += s*rv1[k];
					}
				}
				for (k=l-1;k<n;k++){
					u[i][k] *= scale;
				}
			}
		}
		anorm=MAX(anorm,(abs(w[i])+abs(rv1[i])));
	}
//
//  Accumulation of right-hand transformations
//
	for (i=n-1;i>=0;i--) {
		if (i < n-1) {
			if (g != 0.0) {
				for (j=l;j<n;j++){
					v[j][i]=(u[i][j]/u[i][l])/g;
				}
				for (j=l;j<n;j++) {
					s=0.0;
					for (k=l;k<n;k++){
						s += u[i][k]*v[k][j];
					}
					for (k=l;k<n;k++){
						v[k][j] += s*v[k][i];
					}
				}
			}
			for (j=l;j<n;j++){
				v[i][j]=v[j][i]=0.0;
			}
		}
		v[i][i]=1.0;
		g=rv1[i];
		l=i;
	}
//
//  Accumulation of left-hand transformations
//
	for (i=MIN(m,n)-1;i>=0;i--) {
		l=i+1;
		g=w[i];
		for (j=l;j<n;j++){
			u[i][j]=0.0;
		}
		if (g != 0.0) {
			g=1.0/g;
			for (j=l;j<n;j++) {
				for (s=0.0,k=l;k<m;k++){
					s += u[k][i]*u[k][j];
				}
				f=(s/u[i][i])*g;
				for (k=i;k<m;k++){
					u[k][j] += f*u[k][i];
				}
			}
			for (j=i;j<m;j++){
				u[j][i] *= g;
			}
		} else{
			for (j=i;j<m;j++){
				u[j][i]=0.0;
			}
		}
		++u[i][i];	// not seen this before but it is the same as x++ where x happens to be a matrix entry
	}
//
// Diagonalisation of the bi-diagonal form
//
	for (k=n-1;k>=0;k--) {
		for (its=0;its<30;its++) {
			flag=true;
			for (l=k;l>=0;l--) {
				nm=l-1;
				if (l == 0 || abs(rv1[l]) <= eps*anorm) {
					flag=false;
					break;
				}
				if (abs(w[nm]) <= eps*anorm) break;
			}

			if (flag) {
				c=0.0;
				s=1.0;
				for (i=l;i<k+1;i++) {
					f=s*rv1[i];
					rv1[i]=c*rv1[i];
					if (abs(f) <= eps*anorm) break;
					g=w[i];
					h=pythag(f,g);
					w[i]=h;
					h=1.0/h;
					c=g*h;
					s = -f*h;
					for (j=0;j<m;j++) {
						y=u[j][nm];
						z=u[j][i];
						u[j][nm]=y*c+z*s;
						u[j][i]=z*c-y*s;
					}
				}
			}
// !flag
			z=w[k];
			if (l == k) {
				if (z < 0.0) {
					w[k] = -z;
					for (j=0;j<n;j++){
						v[j][k] = -v[j][k];
					}
				}
				break;
			}
// Convergence test
			if (its == 29){
				throw("no convergence in 30 svdcmp iterations");
			}
			x=w[l];
			nm=k-1;
			y=w[nm];
			g=rv1[nm];
			h=rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			g=pythag(f,1.0);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
//
// next QR transform
//
			c=s=1.0;
			for (j=l;j<=nm;j++) {
				i=j+1;
				g=rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=pythag(f,h);
				rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g=g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=0;jj<n;jj++) {
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag(f,h);
				w[j]=z;
				if (z) {
					z=1.0/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=0;jj<m;jj++) {
					y=u[jj][j];
					z=u[jj][i];
					u[jj][j]=y*c+z*s;
					u[jj][i]=z*c-y*s;
				}
			}
			rv1[l]=0.0;
			rv1[k]=f;
			w[k]=x;
		}
	}
}
//
//----------------------------------------------------------------------
//
void SVD_AS::reorder() 
{
//
// This routine re-orders the singular values and corresponding U & V vectors
// in descending order to simplify the other functions range() and nullity()
// to operate efficiently, as far as I can see this did not exist back then but neither did
// range() and nullity(). The book only covers the user interface
// so this function is not discussed.
//
	Int i,j,k,s,inc=1;
	Doub sw;
	VecDoub su(m), sv(n);
	do { inc *= 3; inc++; } while (inc <= n);
	do {
		inc /= 3;
		for (i=inc;i<n;i++) {
			sw = w[i];			// the swap sigma value
			for (k=0;k<m;k++){
				su[k] = u[k][i];	// save the swap column in U
			}
			for (k=0;k<n;k++){
				sv[k] = v[k][i];	// save the swap column in V
			}
			j = i;
			while (w[j-inc] < sw) {		// if W[j-inc] less than swap
				w[j] = w[j-inc];		// swap it down
				for (k=0;k<m;k++){
					u[k][j] = u[k][j-inc];	// do the same for the column of U
				}
				for (k=0;k<n;k++){
					v[k][j] = v[k][j-inc];	// and V
				}
				j -= inc;
				if (j < inc) break;
			}
			w[j] = sw;

			for (k=0;k<m;k++){
				u[k][j] = su[k];			// now back sub the saved U
			}
			for (k=0;k<n;k++){
				v[k][j] = sv[k];			// and V
			}
		}
	} while (inc > 1);
//
// not entirely sure what this negation of vectors is for? esthetic? 
// the way it is carried out it should be OK but why do it
//
	for (k=0;k<n;k++) {
		s=0;
		for (i=0;i<m;i++) {
			if (u[i][k] < 0.){
				s++;
			}
		}
		for (j=0;j<n;j++){
			if (v[j][k] < 0.){
				s++;
			}
		}
		if (s > (m+n)/2) {		// if more than 1/2 of U & V the vectors are -ve 
			for (i=0;i<m;i++){
				u[i][k] = -u[i][k];	// negate them
			}
			for (j=0;j<n;j++){
				v[j][k] = -v[j][k];
			}
		}
	}
}
//
//----------------------------------------------------------------------
//
void SVD_AS::solve(VecDoub_I &b, VecDoub_O &x, Doub thresh) 
{
/*
C This function (and the overload below) finds the solution to the 
C singular valued inversion problem:  A*x = b 
C after SVD_AS::decompose() has been called.
C
C  Arguments :
C       b                 Input right hand side of the equation
C       x                 Output or solution vector
C       thresh			  Sets the cutoff Threshold for non-zero singular-values
C                         below this value they are effectively zeroed. If thresh<0 (-1)
C						  thresh is calculated based on EPSILON for the given computer
C
C Source of original code : Numerical Recipies by W. Press, B.P. Flannery and others
C                          (C) Cambridge University Press 1986 and 3rd Ed. 2007 .
C User Notes:
C
C  Solves the inversion problem A*X = B for a vector X, where A is specified
C  via the decomposition matrices U,V & W. These matrices are calculated
C  by decompose(). M and N are the logical dimensions of A and will be equal for
C  square matrices. No input quantities are destroyed so the routine may
C  be called sequentially with different B's. M must be greater than or equal
C  to N. See decompose.
C  This is equivant to X = [A^-1]*b where [A^-1] = V*[1/W]*U^T
C  If A is singular the corresponding [1/W] is set zero and the subset of
C  valid X is returned, the remaining elements are zero
C
*/
	Int i,j,jj;
	Doub s;
	if (b.size() != m || x.size() != n) throw("SVD_AS::solve bad sizes");
	VecDoub tmp(n);
	tsh = (thresh >= 0. ? thresh : 0.5*sqrt(m+n+1.)*w[0]*eps);
	for (j=0;j<n;j++) {
		s=0.0;
		if (w[j] > tsh) {
			for (i=0;i<m;i++){
				s += u[i][j]*b[i];
			}
			s /= w[j];
		}
		tmp[j]=s;
	}
	for (j=0;j<n;j++) {
		s=0.0;
		for (jj=0;jj<n;jj++){
			s += v[j][jj]*tmp[jj];
		}
		x[j]=s;
	}
}
//
//----------------------------------------------------------------------
//
void SVD_AS::solve(MatDoub_I &b, MatDoub_O &x, Doub thresh)
{
//
// This overload solves the same system a.x = b as the above variant
// but this time with b a matrix. See above for further details
//
	int i,j,p=b.ncols();
	if (b.nrows() != m || x.nrows() != n || x.ncols() != p)
		throw("SVD_AS::solve bad sizes");
	VecDoub xx(n),bcol(m);
	for (j=0;j<p;j++) {
		for (i=0;i<m;i++) bcol[i] = b[i][j];
		solve(bcol,xx,thresh);
		for (i=0;i<n;i++) x[i][j] = xx[i];
	}
}
Int SVD_AS::rank(Doub thresh = -1.) {
// returns the rank of A given thresh
	Int j,nr=0;
	tsh = (thresh >= 0. ? thresh : 0.5*sqrt(m+n+1.)*w[0]*eps);
	for (j=0;j<n;j++) if (w[j] > tsh) nr++;
	return nr;
}
//
//----------------------------------------------------------------------
//
Int SVD_AS::nullity(Doub thresh = -1.) {
// returns the null-space dimension of A
	Int j,nn=0;
	tsh = (thresh >= 0. ? thresh : 0.5*sqrt(m+n+1.)*w[0]*eps);
	for (j=0;j<n;j++) if (w[j] <= tsh) nn++;
	return nn;
}

MatDoub SVD_AS::range(Doub thresh = -1.){
//
// gives the linear sub-space of A as column vectors 
// for which condition numbers are above threshold
//
	Int i,j,nr;
	nr=0;
	MatDoub rnge(m,rank(thresh));
	for (j=0;j<n;j++) {
		if (w[j] > tsh) {
			for (i=0;i<m;i++) {
				rnge[i][nr] = u[i][j];
			}
			nr++;
		}
	}
	return rnge;
}

MatDoub SVD_AS::nullspace(Doub thresh = -1.){
//
// gives the junk sub-space of A as the column vectors of v
// for which the condition numbers are < thresh
//
	Int j,jj,nn;
	nn=0;
	MatDoub nullsp(n,nullity(thresh));
	for (j=0;j<n;j++) {
		if (w[j] <= tsh) {
			for (jj=0;jj<n;jj++) {
				nullsp[jj][nn] = v[jj][j];
			}
			nn++;
		}
	}
	return nullsp;
}


Doub SVD_AS::pythag(const Doub a, const Doub b) {
	Doub absa=abs(a), absb=abs(b);
	return (absa > absb ? absa*sqrt(1.0+SQR(absb/absa)) :
		(absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb))));
}
