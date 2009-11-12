//
// PD - Controller
//
int updatePD(long targetPosition, long currentPosition, struct PD *p) {
  long dTerm = (p->D * (currentPosition - p->lastPosition)) / 100;
  p->lastPosition = currentPosition;
  return ((p->P * (targetPosition - currentPosition)) / 100) + dTerm;
}
