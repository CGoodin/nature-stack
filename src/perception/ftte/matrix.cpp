/*
Non-Commercial License - Mississippi State University Off-Road Traversability Algorithm

REPO: https://gitlab.com/cgoodin/off_road_traversability

CONTACT: cgoodin@cavs.msstate.edu

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

CITATION:
Goodin, C., Dabbiru, L., Hudson, C., Mason, G., Carruth, D., & Doude, M. (2021, April).
Fast terrain traversability estimation with terrestrial lidar in off-road autonomous navigation.
In Unmanned Systems Technology XXIII (Vol. 11758, p. 117580O). International Society for Optics and Photonics.

NOTICE:
Do not share or distribute. Software is authorized for use only by the approved recepient.

Copyright 2022 (C) Mississippi State University
*/
#include "nature/perception/ftte/matrix.h"
#include <iostream>

namespace traverselib{

Matrix::Matrix(){};

Matrix::Matrix(int nrows, int ncols){
  Resize(nrows, ncols);
}

Matrix::~Matrix(){};

Matrix::Matrix (const Matrix &m){
  Resize(m.GetNumRows(), m.GetNumCols());
  elements_ = m.elements_;
};

Matrix Matrix::Transpose(){
  Matrix m(ncols_, nrows_);
  for (int i=0;i<nrows_; i++){
    for (int j=0;j<ncols_; j++){
      m(j,i) = GetElement(i,j);
    }
  }
  return m;
}

Matrix Matrix::GetMinor(int i, int j){
  Matrix m(nrows_-1, ncols_-1);
  if (i>=0 && i<nrows_ && j>=0 && j<ncols_){
    for (int ii=0;ii<nrows_; ii++){
      for (int jj=0;jj<ncols_; jj++){
	if(ii!=i && jj!=j){
	  int rn = ii;
	  int cn = jj;
	  if (ii>i) rn = ii-1;
	  if (jj>j) cn = jj-1;
	  m(rn,cn) = GetElement(ii,jj);
	}
      }
    }
  }
  else {
    std::cerr<<"Warning: Attempted to take minor using out of bounds indices "<<i<<" "<<j<<std::endl;
  }
  return m;
}

Matrix Matrix::Inverse(){
  Matrix inv;
  inv.Resize(nrows_,ncols_);
  if (nrows_==ncols_){
    if (ncols_==2){
      double d = Determinant();
      inv(0,0) = GetElement(1,1)/d;
      inv(1,1) = GetElement(0,0)/d;
      inv(0,1) = -1.0*GetElement(0,1)/d;
      inv(1,0) = -1.0*GetElement(1,0)/d;
    }
    else {
      Matrix matrix_of_minors(nrows_, ncols_);
      for (int i=0; i<nrows_; i++){
            for (int j=0;j<ncols_; j++ ){
	      Matrix minor = GetMinor(i,j);
	      matrix_of_minors(i,j) =  PermuteSign(i+j)*minor.Determinant();
            }
      }
      
      Matrix a = matrix_of_minors.Transpose();
      //Matrix inv(a);
      double d = Determinant();
      for (int i=0; i<nrows_; i++){
	for (int j=0;j<ncols_; j++ ){
	  inv(i,j) = a(i,j)/d;
	}
      }
    }	
  }
  return inv;
} // inverse

double Matrix::Determinant(){
  double d = 0.0;
  if (nrows_==ncols_){
    if (nrows_==2){
      d = GetElement(0,0)*GetElement(1,1) - GetElement(0,1)*GetElement(1,0);
    }
    else{
      for (int j=0; j<nrows_; j++){
	Matrix minor = GetMinor(0,j);
	d = d + PermuteSign(j)*GetElement(0,j)*minor.Determinant();
      }
    }
        }
  else {
    std::cerr<<"Warning: Attempted to take the determinant of a "<<nrows_<<" by "<<ncols_<<" matrix."<<std::endl;
  }
  return d;
    }

void Matrix::Resize(int nrows, int ncols){
  nrows_ = nrows;
  ncols_ = ncols;
  elements_.resize(nrows*ncols,0.0);
}

void Matrix::Print(){
  for (int i = 0; i<nrows_; i++){
    for (int j = 0; j<ncols_; j++){
      std::cout<<elements_[GetIndex(i,j)]<<" ";
    }
    std::cout<<std::endl;
  }
  std::cout<<std::endl;
}

bool Matrix::CompareSize(const Matrix& a)const {
  if (!(a.GetNumCols()==ncols_ && a.GetNumRows()==nrows_) ){
    return false;
  }
  else {
    return true;
  }
}

int Matrix::GetIndex(int i, int j) const {
  int index = j + i*ncols_;
  if (index<0 || index>=(int)elements_.size()){
    std::cerr<<"Warning: Requested array index out of bounds: "<<index<<std::endl;
    index = 0;
  }
  return index;
}

int Matrix::PermuteSign(int i){
  int s = 1;
  int m = i%2;
  if (m==1) s = -1;
  return s;
}

} //namespace traverslib
