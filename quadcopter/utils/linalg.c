


//*****************************************************************************
//
// 2x2 matrix inverse
//
//*****************************************************************************
void Inv2(float* a, float* ia) {
  float detA = a[0]*a[3] - a[1]*a[2];
  ia[0] =  detA*a[3];
  ia[1] = -detA*a[1];
  ia[2] = -detA*a[2];
  ia[3] =  detA*a[0];
}

//*****************************************************************************
//
// 2x2 matrix dot multiplication
//
//*****************************************************************************
void Dot2(float* a, float* b, float *c) {
  c[0] = a[0] * b[0] + a[1] * b[2];
  c[1] = a[0] * b[1] + a[1] * b[3];
  c[2] = a[2] * b[0] + a[3] * b[2];
  c[3] = a[2] * b[1] + a[3] * b[3];
}
